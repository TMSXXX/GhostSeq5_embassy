use core::f32::consts::PI;
use core::fmt::Write;
use defmt::info;
use embassy_executor::task;
use embassy_futures::select::{Either, select};
use embassy_stm32::Peri;
use embassy_stm32::adc::Adc;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pin;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{ADC1, PB0};
use embassy_stm32::peripherals::{PA8, PA9, PA10};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::String;
use micromath::F32Ext;

use crate::{
    AMP_CHANNEL, AUDIO_CHANNEL, AudioCommand, WAVE_PARAMS_CHANNEL, WAVE_TABLE_SIZE, WaveParams,
    Waveform, get_sawtooth_table, get_sine_table, get_square_table, get_triangle_table,
};

// --- (新!) FM 合成器的参数 ---
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FmParams {
    pub index: f32, // “金属感” (0.0 - 10.0)
    pub ratio: f32, // “音色” (0.5 - 5.0)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DrumSample {
    Kick,
    Snare,
    Hat,
}

const NOTE_FREQUENCIES: [f32; 8] = [
    261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88, 523.25,
];

const SEMITONE_MULTIPLIERS: [f32; 13] = [
    0.7491, 0.7937, 0.8409, 0.8909, 0.9439, 1.0, 1.0595, 1.1225, 1.1892, 1.2599, 1.3348, 1.4142,
    1.4983,
];
const SEMITONE_SHIFT_OFFSET: i8 = 5;

pub type OledText = String<16>;

// --- (已回退) Channel 定义 ---
// (移除了 MOD_ENV_CHANNEL 和 FILTER_PARAMS_CHANNEL)
pub static FM_PARAM_CHANNEL: Channel<CriticalSectionRawMutex, FmParams, 2> = Channel::new(); // (已恢复)
pub static ENCODER_ROTARY_CHANNEL: Channel<CriticalSectionRawMutex, i8, 4> = Channel::new();
pub static ENCODER_SWITCH_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
pub static DRUM_CHANNEL: Channel<CriticalSectionRawMutex, DrumSample, 4> = Channel::new();
pub static POT1_CHANNEL: Channel<CriticalSectionRawMutex, u16, 4> = Channel::new();
pub static HAAS_STATE_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
pub static UI_DASHBOARD_CHANNEL: Channel<CriticalSectionRawMutex, UiState, 2> = Channel::new();

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UiState {
    pub octave: usize,
    pub semitone: i8,
    pub fm_index: f32, // (现在它又变回 FM Index 了)
    pub carrier_wave: Waveform,
    pub mod_wave: Waveform,
    pub is_shift_held: bool,
    pub is_haas_active: bool,
    pub mode: SequencerMode,
    pub step: usize,
}

// ... (SequencerMode, NoteData, Envelope (及 impl) ... 不变) ...
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum SequencerMode {
    Stop,
    Play,
    Record,
}
#[derive(Debug, Clone, Copy, PartialEq)]
struct NoteData {
    key_code: u8,
    octave: usize,
    semitone: i8,
    is_sharp: bool,
}
#[derive(Debug, Clone, Copy, PartialEq)]
enum EnvelopeStage {
    Idle,
    Attack,
    Decay,
    Sustain,
    Release,
}
#[derive(Debug, Clone, Copy, PartialEq)]
struct Envelope {
    stage: EnvelopeStage,
    current_value: f32,
    attack_ticks: f32,
    decay_ticks: f32,
    sustain_level: f32,
    release_ticks: f32,
    attack_step: f32,
    decay_step: f32,
    release_step: f32,
    release_start_value: f32,
}
impl Envelope {
    fn new() -> Self {
        Self {
            stage: EnvelopeStage::Idle,
            current_value: 0.0,
            attack_ticks: 2.0,
            decay_ticks: 100.0,
            sustain_level: 0.0,
            release_ticks: 40.0,
            attack_step: 0.0,
            decay_step: 0.0,
            release_step: 0.0,
            release_start_value: 0.0,
        }
    }
    fn note_on(&mut self) {
        self.stage = EnvelopeStage::Attack;
        self.attack_step = 1.0 / self.attack_ticks;
        self.decay_step = (self.sustain_level - 1.0) / self.decay_ticks;
    }
    fn note_off(&mut self) {
        if self.stage != EnvelopeStage::Idle {
            self.stage = EnvelopeStage::Release;
            self.release_start_value = self.current_value;
            self.release_step = -self.release_start_value / self.release_ticks;
        }
    }
    fn tick(&mut self) -> f32 {
        match self.stage {
            EnvelopeStage::Idle => {
                self.current_value = 0.0;
            }
            EnvelopeStage::Attack => {
                self.current_value += self.attack_step;
                if self.current_value >= 1.0 {
                    self.current_value = 1.0;
                    self.stage = EnvelopeStage::Decay;
                }
            }
            EnvelopeStage::Decay => {
                self.current_value += self.decay_step;
                if self.current_value <= self.sustain_level {
                    self.current_value = self.sustain_level;
                    self.stage = EnvelopeStage::Sustain;
                    if self.sustain_level == 0.0 {
                        self.stage = EnvelopeStage::Idle;
                    }
                }
            }
            EnvelopeStage::Sustain => {
                self.current_value = self.sustain_level;
            }
            EnvelopeStage::Release => {
                self.current_value += self.release_step;
                if self.current_value <= 0.0 {
                    self.current_value = 0.0;
                    self.stage = EnvelopeStage::Idle;
                }
            }
        }
        self.current_value
    }
    fn is_idle(&self) -> bool {
        matches!(self.stage, EnvelopeStage::Idle)
    }
}

// (calculate_final_frequency 不变)
fn calculate_final_frequency(
    base_frequency: f32,
    is_sharp: bool,
    semitone_shift: i8,
    octave_scale: usize,
) -> f32 {
    const OCTAVE_MULTI: [f32; 5] = [0.25, 0.5, 1., 2., 4.];
    const SEMITONE_UP: f32 = 1.0594635;

    let semitone_index = (semitone_shift + SEMITONE_SHIFT_OFFSET) as usize;
    let mut final_freq = base_frequency * SEMITONE_MULTIPLIERS[semitone_index];

    if is_sharp {
        final_freq *= SEMITONE_UP;
    }
    final_freq *= OCTAVE_MULTI[octave_scale];
    final_freq
}

#[task]
pub async fn control_task(keys: [[Peri<'static, AnyPin>; 4]; 2]) {
    info!("Control task (P7) started!");

    // --- 1. 键盘状态 (不变) ---
    let [rows, cols] = keys.map(|line| line);
    let mut rows = rows.map(|pin| Output::new(pin, Level::High, Speed::Low));
    let cols = cols.map(|pin| Input::new(pin, Pull::Up));
    let mut last_key_state: [bool; 16] = [false; 16];

    // --- 2. 合成器状态 (不变) ---
    let mut current_frequency = 0.0f32;
    let mut is_sharp_active = false;
    let mut octave_scale: usize = 2;
    let mut semitone_shift: i8 = 0;
    let mut note_keys_pressed: u8 = 0;

    // --- 3. 音色状态 (已回退) ---
    let mut current_params = FmParams {
        index: 0.0,
        ratio: 2.0,
    }; // (已恢复)
    let mut max_fm_index: f32 = 2.0; // (已恢复)
    let mut wave_params = WaveParams {
        carrier_wave: Waveform::Triangle,
        mod_wave: Waveform::Square,
    };
    let mut is_haas_active = false;
    let mut is_shift_held = false;

    // --- 4. 包络 (已回退) ---
    let mut fm_envelope = Envelope::new(); // (已恢复)
    fm_envelope.attack_ticks = 0.2;
    fm_envelope.decay_ticks = 15.0;
    fm_envelope.sustain_level = 0.0;
    fm_envelope.release_ticks = 6.0;
    let mut amp_envelope = Envelope::new();
    // (回到 Pluck 音色)
    amp_envelope.attack_ticks = 0.2;
    amp_envelope.decay_ticks = 40.0;
    amp_envelope.sustain_level = 0.5;
    amp_envelope.release_ticks = 25.0;

    // --- 6. Sequencer (音序器) 状态 (32步) ---
    let mut sequencer_mode = SequencerMode::Stop;
    let mut current_step: usize = 0;
    let mut bpm: f32 = 120.0;
    let mut step_duration = Duration::from_millis(125);
    let mut last_tick_time = Instant::now();
    let mut sequence: [Option<NoteData>; 32] = [None; 32];

    // --- 7. 功能键 ID (不变) ---
    const CARRIER_WAVE_ID: u8 = 11;
    const MOD_WAVE_ID: u8 = 10;
    const HAAS_TOGGLE_ID: u8 = 12;
    const PLAY_STOP_ID: u8 = 15;
    const KICK_DRUM_ID: u8 = 9;
    const RECORD_ID: u8 = 14;

    // (发送初始状态 ... 已回退)
    let _ = FM_PARAM_CHANNEL.try_send(current_params); // (已恢复)
    let _ = AMP_CHANNEL.try_send(0.0);
    let _ = WAVE_PARAMS_CHANNEL.try_send(wave_params);
    let _ = HAAS_STATE_CHANNEL.try_send(is_haas_active);

    // -----------------------------------------------------------------
    // 8. P7 主循环 (使用 Ticker)
    // -----------------------------------------------------------------
    let mut ticker = Ticker::every(Duration::from_millis(5));

    loop {
        // --- 8A. 检查所有通道 (已回退) ---
        if let Ok(rotation) = crate::synth::ENCODER_ROTARY_CHANNEL.try_receive() {
            if is_shift_held {
                semitone_shift += match rotation > 0 {
                    true => 1,
                    false => -1,
                };
                semitone_shift = semitone_shift.clamp(-5, 7);
                // (Resonance 已移除)
            } else {
                if rotation > 0 {
                    if octave_scale < 4 {
                        octave_scale += 1;
                    }
                } else {
                    if octave_scale > 0 {
                        octave_scale -= 1;
                    }
                }
            }
        }
        if let Ok(pressed) = crate::synth::ENCODER_SWITCH_CHANNEL.try_receive() {
            is_shift_held = pressed;
        }
        if let Ok(val) = crate::synth::POT1_CHANNEL.try_receive() {
            // (已恢复) 电位器控制 FM Index
            max_fm_index = (val as f32 / 4095.0) * 10.0;
        }

        // --- 8B. 扫描键盘 (不变) ---
        let mut current_key_state: [bool; 16] = [false; 16];
        for (r, row) in rows.iter_mut().enumerate() {
            row.set_low();
            for (c, col) in cols.iter().enumerate() {
                if col.is_low() {
                    current_key_state[r * 4 + c] = true;
                }
            }
            row.set_high();
        }

        // --- 8C. 处理按键逻辑 (已回退) ---
        for i in 0..16 {
            let key_pressed = current_key_state[i] && !last_key_state[i];
            let key_released = !current_key_state[i] && last_key_state[i];
            let key_code = i as u8;

            if key_pressed {
                match key_code {
                    0..=7 => {
                        note_keys_pressed += 1;
                        let note_data = NoteData {
                            key_code,
                            octave: octave_scale,
                            semitone: semitone_shift,
                            is_sharp: is_sharp_active,
                        };
                        if sequencer_mode == SequencerMode::Record {
                            sequence[current_step] = Some(note_data);
                        }
                        let base_frequency = NOTE_FREQUENCIES[key_code as usize];
                        current_frequency = calculate_final_frequency(
                            base_frequency,
                            is_sharp_active,
                            semitone_shift,
                            octave_scale,
                        );
                        if note_keys_pressed == 1 {
                            fm_envelope.note_on(); // (已恢复)
                            amp_envelope.note_on();
                        }
                        let _ = AUDIO_CHANNEL.try_send(AudioCommand::Play(current_frequency));
                    }

                    // (功能键 不变)
                    CARRIER_WAVE_ID => {
                        wave_params.carrier_wave = match wave_params.carrier_wave {
                            Waveform::Sine => Waveform::Triangle,
                            Waveform::Triangle => Waveform::Sawtooth,
                            Waveform::Sawtooth => Waveform::Square,
                            Waveform::Square => Waveform::Sine,
                        };
                        let _ = WAVE_PARAMS_CHANNEL.try_send(wave_params);
                    }
                    KICK_DRUM_ID => {
                        let _ = DRUM_CHANNEL.try_send(DrumSample::Kick);
                    }
                    MOD_WAVE_ID => {
                        wave_params.mod_wave = match wave_params.mod_wave {
                            Waveform::Sine => Waveform::Triangle,
                            Waveform::Triangle => Waveform::Sawtooth,
                            Waveform::Sawtooth => Waveform::Square,
                            Waveform::Square => Waveform::Sine,
                        };
                        let _ = WAVE_PARAMS_CHANNEL.try_send(wave_params);
                    }
                    HAAS_TOGGLE_ID => {
                        is_haas_active = !is_haas_active;
                        let _ = HAAS_STATE_CHANNEL.try_send(is_haas_active);
                    }
                    PLAY_STOP_ID => {
                        sequencer_mode = match sequencer_mode {
                            SequencerMode::Stop => {
                                last_tick_time = Instant::now();
                                current_step = 31;
                                SequencerMode::Play
                            }
                            _ => {
                                let _ = AUDIO_CHANNEL.try_send(AudioCommand::Stop);
                                SequencerMode::Stop
                            }
                        };
                    }
                    RECORD_ID => {
                        if is_shift_held {
                            sequence = [None; 32];
                            sequencer_mode = SequencerMode::Stop;
                            current_step = 0;
                            let _ = AUDIO_CHANNEL.try_send(AudioCommand::Stop);
                            amp_envelope.note_off();
                            fm_envelope.note_off(); // (已恢复)
                        } else {
                            if sequencer_mode == SequencerMode::Record {
                                sequencer_mode = SequencerMode::Play;
                            } else {
                                sequencer_mode = SequencerMode::Record;
                            }
                        }
                    }
                    _ => {}
                }
            } else if key_released {
                if key_code <= 7 && last_key_state[i] {
                    if note_keys_pressed > 0 {
                        note_keys_pressed -= 1;
                    }

                    if note_keys_pressed == 0 {
                        amp_envelope.note_off();
                        fm_envelope.note_off(); // (已恢复)
                    }
                }
            }
        }
        last_key_state = current_key_state;

        // --- 8D. BPM 时钟 (32步) ---
        let now = Instant::now();
        if (sequencer_mode != SequencerMode::Stop)
            && (now.duration_since(last_tick_time) >= step_duration)
        {
            last_tick_time = now;
            current_step = (current_step + 1) % 32;

            if let Some(note) = sequence[current_step] {
                let base_frequency = NOTE_FREQUENCIES[note.key_code as usize];
                let final_freq = calculate_final_frequency(
                    base_frequency,
                    note.is_sharp,
                    note.semitone,
                    note.octave,
                );

                fm_envelope.note_on(); // (已恢复)
                amp_envelope.note_on();
                let _ = AUDIO_CHANNEL.try_send(AudioCommand::Play(final_freq));
            } else if note_keys_pressed == 0 {
                amp_envelope.note_off();
                fm_envelope.note_off(); // (已恢复)
            }
        }

        // --- 8E. 运行包络 & 发送参数 (已回退) ---
        let fm_env_val = fm_envelope.tick(); // (已恢复)
        let amp_env_val = amp_envelope.tick();

        current_params.index = fm_env_val * max_fm_index; // (已恢复)

        let _ = FM_PARAM_CHANNEL.try_send(current_params); // (已恢复)
        let _ = AMP_CHANNEL.try_send(amp_env_val);

        // --- 8F. 停止播放逻辑 (不变) ---
        if amp_envelope.is_idle() && note_keys_pressed == 0 && sequencer_mode == SequencerMode::Stop
        {
            let _ = AUDIO_CHANNEL.try_send(AudioCommand::Stop);
        }

        // --- 8G. 发送 UI 状态包 (已回退) ---
        let ui_state = UiState {
            octave: octave_scale,
            semitone: semitone_shift,
            fm_index: max_fm_index, // (已恢复: 显示最大值)
            carrier_wave: wave_params.carrier_wave,
            mod_wave: wave_params.mod_wave,
            is_shift_held: is_shift_held,
            is_haas_active: is_haas_active,
            mode: sequencer_mode,
            step: current_step,
        };
        let _ = UI_DASHBOARD_CHANNEL.try_send(ui_state);

        // --- 9. Await 5ms ---
        ticker.next().await;
    }
}

#[task]
pub async fn adc_task(mut adc: Adc<'static, ADC1>, mut pin: Peri<'static, PB0>) {
    info!("ADC task (P15) started!");
    loop {
        let value = adc.blocking_read(&mut pin);
        let _ = POT1_CHANNEL.try_send(value);
        Timer::after_millis(20).await;
    }
}

// (encoder_task 已修复)
#[task]
pub async fn encoder_task(
    mut clk_a: ExtiInput<'static>,
    mut dt_b: ExtiInput<'static>,
    mut sw: ExtiInput<'static>,
) {
    info!("Encoder task (P7) started!");

    // (新!) 积分器去抖状态
    let mut sw_stable_state = true; // true = 松开 (PULL_UP)
    let mut sw_last_reading = true;
    let mut sw_stable_counter = 0;
    const DEBOUNCE_TICKS: u8 = 4; // 4 * 5ms = 20ms

    loop {
        // (新!) 我们现在用一个 5ms 的 Ticker 和 select 来 *同时* 处理旋转和按键
        let clk_fall = clk_a.wait_for_falling_edge();
        let sw_edge = sw.wait_for_any_edge();
        let tick = Timer::after_millis(5); // 我们的 5ms 去抖时钟

        match select(clk_fall, select(sw_edge, tick)).await {
            // --- 旋转 (Either::First(...)) ---
            Either::First(_) => {
                // clk_fall 触发了
                if dt_b.is_low() {
                    let _ = ENCODER_ROTARY_CHANNEL.try_send(1); // 顺时针
                } else {
                    let _ = ENCODER_ROTARY_CHANNEL.try_send(-1); // 逆时针
                }
            }

            // --- 按键或 Ticker (Either::Second(...)) ---
            Either::Second(either_sw_or_tick) => {
                match either_sw_or_tick {
                    // --- 按键中断触发 (Either::First(...)) ---
                    Either::First(_) => {
                        // sw_edge 触发了
                        // (中断触发了，我们重置计数器，等待 Ticker 确认)
                        sw_stable_counter = 0;
                    }

                    // --- 5ms Ticker 触发 (Either::Second(...)) ---
                    Either::Second(_) => {
                        // tick 触发了
                        // --- 在这里运行我们的“积分器”去抖 ---
                        let sw_curr_reading = sw.is_high();
                        if sw_curr_reading == sw_last_reading {
                            // 状态看起来稳定...
                            if sw_stable_counter < DEBOUNCE_TICKS {
                                sw_stable_counter += 1;
                                if sw_stable_counter == DEBOUNCE_TICKS {
                                    // 状态 *确认* 稳定了 20ms
                                    if sw_curr_reading != sw_stable_state {
                                        // 这是一个 *新* 的稳定状态，发送它
                                        sw_stable_state = sw_curr_reading;
                                        let pressed = !sw_stable_state;
                                        let _ = ENCODER_SWITCH_CHANNEL.try_send(pressed);
                                    }
                                }
                            }
                        } else {
                            // 状态不稳定 (正在弹跳)，重置计数器
                            sw_stable_counter = 0;
                        }
                        sw_last_reading = sw_curr_reading;
                    }
                }
            }
        }
    }
}
