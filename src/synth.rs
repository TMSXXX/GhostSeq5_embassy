// (synth.rs)

use core::f32::consts::PI;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use defmt::info;
use embassy_executor::task;
use embassy_futures::select::{Either, select};
use embassy_stm32::Peri;
use embassy_stm32::adc::Adc;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pin;
use embassy_stm32::gpio::{AnyPin, Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{ADC1, PB0, PB1};
use embassy_stm32::peripherals::{PA8, PA9, PA10};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::String;
use micromath::F32Ext;

use crate::wavetable::{
    WAVE_TABLE_SIZE, WaveParams, Waveform, get_sawtooth_table, get_sine_table, get_square_table,
    get_triangle_table,
};
use crate::{
    AMP_CHANNEL, AUDIO_CHANNEL, AudioCommand, RECORD_COMMAND_CHANNEL, USER_SAMPLE_LEN,
    USER_SAMPLE_READY, WAVE_PARAMS_CHANNEL,
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
    User,
}

// --- (新!) ADSR 控制状态定义 ---
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EnvParam {
    // 被编码器调节的参数 (A, D, S, R)
    Attack,
    Decay,
    Sustain,
    Release,
    MasterDrive,
    Bitcrusher,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ActiveEnv {
    // 当前被编辑的包络焦点 (Amp 或 FM)
    Amp,
    Fm,
}

const NOTE_FREQUENCIES: [f32; 12] = [
    // ... (NOTE_FREQUENCIES 不变) ...
    261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30, 440.00, 466.16, 493.88,
];

const SEMITONE_MULTIPLIERS: [f32; 13] = [
    // ... (SEMITONE_MULTIPLIERS 不变) ...
    0.7491, 0.7937, 0.8409, 0.8909, 0.9439, 1.0, 1.0595, 1.1225, 1.1892, 1.2599, 1.3348, 1.4142,
    1.4983,
];
const SEMITONE_SHIFT_OFFSET: i8 = 5;

pub type OledText = String<16>;

// --- Channel 定义 ---
pub static FM_PARAM_CHANNEL: Channel<CriticalSectionRawMutex, FmParams, 2> = Channel::new();
pub static ENCODER_ROTARY_CHANNEL: Channel<CriticalSectionRawMutex, i8, 4> = Channel::new();
pub static ENCODER_SWITCH_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
pub static DRUM_CHANNEL: Channel<CriticalSectionRawMutex, DrumSample, 4> = Channel::new();
pub static POT1_CHANNEL: Channel<CriticalSectionRawMutex, u16, 4> = Channel::new();
pub static HAAS_STATE_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
pub static MASTER_DRIVE_CHANNEL: Channel<CriticalSectionRawMutex, f32, 2> = Channel::new();
pub static BITCRUSH_CHANNEL: Channel<CriticalSectionRawMutex, i8, 2> = Channel::new();
pub static REVERSE_STATE_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
pub static SAMPLE_PITCH_CHANNEL: Channel<CriticalSectionRawMutex, f32, 2> = Channel::new();
pub static IS_RECORDING: AtomicBool = AtomicBool::new(false);

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
}

// --- (新!) 允许多个鼓声在同一步 ---
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct DrumTracks {
    kick: bool,
    snare: bool,
    hat: bool,
    sample: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct StepData {
    note: Option<NoteData>,
    drums: DrumTracks, // <-- (已修改)
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ControlState {
    Keyboard,
    Drums,
    Dsp,
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
fn calculate_final_frequency(base_frequency: f32, semitone_shift: i8, octave_scale: usize) -> f32 {
    const OCTAVE_MULTI: [f32; 6] = [0.25, 0.5, 1., 2., 4., 8.];

    let semitone_index = (semitone_shift + SEMITONE_SHIFT_OFFSET) as usize;
    let mut final_freq = base_frequency * SEMITONE_MULTIPLIERS[semitone_index];

    final_freq *= OCTAVE_MULTI[octave_scale];
    final_freq
}

#[task]
pub async fn control_task(keys: [[Peri<'static, AnyPin>; 4]; 2], mut led: Output<'static>) {
    info!("Control task (P7) started!");

    let mut control_mode = ControlState::Keyboard;

    // --- 1. 键盘状态 (不变) ---
    let [rows, cols] = keys.map(|line| line);
    let mut rows = rows.map(|pin| Output::new(pin, Level::High, Speed::Low));
    let cols = cols.map(|pin| Input::new(pin, Pull::Up));
    let mut last_key_state: [bool; 16] = [false; 16];

    // --- 2. 合成器状态 (不变) ---

    let mut current_frequency = 0.0f32;
    let mut octave_scale: usize = 2;
    let mut semitone_shift: i8 = 0;
    let mut note_keys_pressed: u8 = 0;
    let mut is_reverse = false;
    let mut sample_playback_speed: f32 = 0.18;

    // --- 3. 音色状态 (已回退) ---
    let mut master_drive: f32 = 1.0;
    let mut bitcrush: i8 = 0;
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

    // --- 4. 包络 (已修改) ---
    let mut fm_envelope = Envelope::new(); // (已恢复)
    fm_envelope.attack_ticks = 0.2;
    fm_envelope.decay_ticks = 10.0;
    fm_envelope.sustain_level = 0.0;
    fm_envelope.release_ticks = 6.0;

    let mut amp_envelope = Envelope::new();
    amp_envelope.attack_ticks = 0.2;
    amp_envelope.decay_ticks = 60.0;
    amp_envelope.sustain_level = 0.1;
    amp_envelope.release_ticks = 10.0;

    let mut active_env = ActiveEnv::Amp; // <-- (新!) 默认调整 Amp 包络
    let mut active_env_param = EnvParam::Attack; // <-- (新!) 默认调整 Attack

    // --- 6. Sequencer (音序器) 状态 (32步) ---
    let mut sequencer_mode = SequencerMode::Stop;
    let mut current_step: usize = 0;
    let mut bpm: f32 = 120.0;
    let mut step_duration = Duration::from_millis(125);
    let mut last_tick_time = Instant::now();
    let mut sequence: [Option<StepData>; 32] = [None; 32];
    let mut stutter_locked_data: Option<StepData> = None;

    // --- 7. 功能键 ID (不变) ---
    const CARRIER_WAVE_ID: u8 = 0;
    const MOD_WAVE_ID: u8 = 1;
    const HAAS_TOGGLE_ID: u8 = 3; // (改为 Key 3)
    const RECORD_START_ID: u8 = 11; // Key 11: 启动/停止录音

    const A_PARAM_ID: u8 = 4; // (Key 4)
    const D_PARAM_ID: u8 = 5; // (Key 5)
    const S_PARAM_ID: u8 = 6; // (Key 6)
    const R_PARAM_ID: u8 = 7; // (Key 7)
    const MASTER_DRIVE_ID: u8 = 8;
    const BITCRUSER_ID: u8 = 9;

    const ENV_TOGGLE_ID: u8 = 2; // (Key 2)

    const KICK_DRUM_ID: u8 = 0;
    const SNARE_DRUM_ID: u8 = 1;
    const HAT_DRUM_ID: u8 = 2;
    const USER_ID: u8 = 3;
    const REVERSE_TOGGLE_ID: u8 = 8;
    const BEAT_REPEAT_TRIGGER_ID: u8 = 9;

    const DSP_MODE_ID: u8 = 12;
    const DRUM_MODE_ID: u8 = 13;
    const RECORD_ID: u8 = 14;
    const PLAY_STOP_ID: u8 = 15;

    // (发送初始状态 ... 已回退)
    let _ = FM_PARAM_CHANNEL.try_send(current_params); // (已恢复)
    let _ = AMP_CHANNEL.try_send(0.0);
    let _ = WAVE_PARAMS_CHANNEL.try_send(wave_params);
    let _ = HAAS_STATE_CHANNEL.try_send(is_haas_active);

    // -----------------------------------------------------------------
    // 8. P7 主循环 (使用 Ticker)
    // -----------------------------------------------------------------
    let mut ticker = Ticker::every(Duration::from_millis(20));
    let mut skip_sequencer_step: Option<usize> = None;

    loop {
        // --- 8A. 检查所有通道 (已修改) ---
        if let Ok(rotation) = crate::synth::ENCODER_ROTARY_CHANNEL.try_receive() {
            info!("Encoder Rotation Received: {}", rotation);

            match control_mode {
                ControlState::Dsp => {
                    // --- 模式 1: DSP (Hybrid 包络调节) ---
                    let scale = if is_shift_held { 0.1 } else { 1.0 }; // Shift 微调

                    // 1. 确定要修改哪个包络 (Amp 或 FM)
                    let env_to_modify = match active_env {
                        ActiveEnv::Amp => &mut amp_envelope,
                        ActiveEnv::Fm => &mut fm_envelope,
                    };

                    // 2. 根据 active_env_param 调整选定的参数
                    match active_env_param {
                        EnvParam::Attack => {
                            env_to_modify.attack_ticks = (env_to_modify.attack_ticks
                                + rotation as f32 * scale)
                                .clamp(0.2, 500.0);
                        }
                        EnvParam::Decay => {
                            env_to_modify.decay_ticks = (env_to_modify.decay_ticks
                                + rotation as f32 * scale * 10.0)
                                .clamp(1.0, 5000.0);
                        }
                        EnvParam::Sustain => {
                            // Sustain Level 是 0.0 到 1.0
                            env_to_modify.sustain_level = (env_to_modify.sustain_level
                                + rotation as f32 * scale * 0.1)
                                .clamp(0.0, 1.0);
                        }
                        EnvParam::Release => {
                            env_to_modify.release_ticks = (env_to_modify.release_ticks
                                + rotation as f32 * scale * 10.0)
                                .clamp(1.0, 600.0);
                        }
                        EnvParam::MasterDrive => {
                            // <-- (新!) Master Drive 调节
                            master_drive =
                                (master_drive + rotation as f32 * scale).clamp(1.0, 11.0);
                            let _ = MASTER_DRIVE_CHANNEL.try_send(master_drive);
                        }
                        EnvParam::Bitcrusher => {
                            bitcrush = (bitcrush + rotation).clamp(0, 15);
                            let _ = BITCRUSH_CHANNEL.try_send(bitcrush);
                        }
                    }
                }

                ControlState::Drums => {
                    // --- 模式 2: DRUMS ---

                    if is_shift_held {
                        // 1. (保持不变) Shift + Encoder = BPM 调节
                        bpm += rotation as f32;
                        bpm = bpm.clamp(60.0, 240.0);
                        let step_ms = 60000.0 / bpm / 4.0;
                        step_duration = Duration::from_millis(step_ms.round() as u64);
                    } else {
                        // 调节步进：0.05 (细腻一点)
                        // 范围：0.1 (慢速怪兽) ~ 2.0 (倍速)
                        let speed_change = rotation as f32 * 0.02;
                        sample_playback_speed =
                            (sample_playback_speed + speed_change).clamp(0.1, 1.0);
                        let _ = SAMPLE_PITCH_CHANNEL.try_send(sample_playback_speed);
                        info!("Sample Speed: {}", sample_playback_speed);
                    }
                }

                ControlState::Keyboard => {
                    // --- 模式 3/4: KEYBOARD / SEQUENCE (Octave/Semitone 调节) ---
                    if is_shift_held {
                        // Shift + Encoder = Semitone
                        semitone_shift += match rotation > 0 {
                            true => 1,
                            false => -1,
                        };
                        semitone_shift = semitone_shift.clamp(-5, 7);
                    } else {
                        // Encoder = Octave
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
            }
        }

        if let Ok(pressed) = crate::synth::ENCODER_SWITCH_CHANNEL.try_receive() {
            is_shift_held = pressed;
        }

        if let Ok(val) = crate::synth::POT1_CHANNEL.try_receive() {
            // (不变) 电位器 始终 控制 FM Index
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

        let is_stutter_pressed = current_key_state[BEAT_REPEAT_TRIGGER_ID as usize];
        if is_stutter_pressed {
            if stutter_locked_data.is_none() {
                // 刚按下：锁定当前步的数据！
                stutter_locked_data = sequence[current_step];
            }
        } else {
            // 松开：解除锁定
            stutter_locked_data = None;
        }

        for i in 0..16 {
            let key_pressed = current_key_state[i] && !last_key_state[i];
            let key_released = !current_key_state[i] && last_key_state[i];
            let key_code = i as u8;

            if key_pressed {
                match key_code {
                    DRUM_MODE_ID => {
                        control_mode = match control_mode {
                            ControlState::Drums => ControlState::Keyboard,
                            _ => ControlState::Drums,
                        }
                    }
                    DSP_MODE_ID => {
                        control_mode = match control_mode {
                            ControlState::Dsp => ControlState::Keyboard,
                            _ => ControlState::Dsp,
                        }
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
                                led.set_high();
                                SequencerMode::Stop
                            }
                        };
                    }
                    RECORD_ID => {
                        if is_shift_held {
                            // --- 上下文敏感的清除逻辑 (保持不变) ---
                            match control_mode {
                                ControlState::Dsp | ControlState::Keyboard => {
                                    // 1. 清除旋律 (Note) 轨道
                                    for step_data in sequence.iter_mut() {
                                        if let Some(step) = step_data {
                                            step.note = None;
                                        }
                                    }
                                    info!("Sequencer: Cleared Note Data.");
                                }
                                ControlState::Drums => {
                                    // 2. 清除鼓点 (Drum) 轨道
                                    for step_data in sequence.iter_mut() {
                                        if let Some(step) = step_data {
                                            step.drums = DrumTracks::default();
                                        }
                                    }
                                    info!("Sequencer: Cleared Drum Data.");
                                }
                            }

                            led.set_high();
                            current_step = 31; // 设置为 31，让 8D. BPM 时钟在下一拍从 0 开始
                        } else {
                            // (原) 录音模式切换逻辑 (保持不变)
                            if sequencer_mode == SequencerMode::Record {
                                sequencer_mode = SequencerMode::Play;
                            } else {
                                sequencer_mode = SequencerMode::Record;
                            }
                        }
                    }

                    0..=11 => {
                        let now = Instant::now();
                        let elapsed = now.duration_since(last_tick_time);

                        // 如果过去的时间超过了步长的一半，说明我们更接近"下一拍"
                        // 此时我们将 target_step 设置为下一拍 (注意处理 31->0 的循环)
                        // 否则，保持当前拍
                        let target_step = if elapsed.as_micros() > (step_duration.as_micros() / 2) {
                            (current_step + 1) & 31
                        } else {
                            current_step
                        };
                        if sequencer_mode == SequencerMode::Record && target_step != current_step {
                            skip_sequencer_step = Some(target_step);
                        }
                        match control_mode {
                            ControlState::Keyboard => {
                                note_keys_pressed += 1;
                                let octave_scale = octave_scale + if is_shift_held { 1 } else { 0 };
                                let note_data = NoteData {
                                    key_code,
                                    octave: octave_scale,
                                    semitone: semitone_shift,
                                };
                                if sequencer_mode == SequencerMode::Record {
                                    let step =
                                        sequence[target_step].get_or_insert_with(StepData::default);
                                    step.note = Some(note_data);
                                }
                                let base_frequency = NOTE_FREQUENCIES[key_code as usize];
                                current_frequency = calculate_final_frequency(
                                    base_frequency,
                                    semitone_shift,
                                    octave_scale,
                                );
                                if note_keys_pressed == 1 {
                                    fm_envelope.note_on();
                                    amp_envelope.note_on();
                                }
                                let _ =
                                    AUDIO_CHANNEL.try_send(AudioCommand::Play(current_frequency));
                            }

                            ControlState::Drums => match key_code {
                                KICK_DRUM_ID => {
                                    if sequencer_mode == SequencerMode::Record {
                                        // (修改) 使用 target_step
                                        let step = sequence[target_step]
                                            .get_or_insert_with(StepData::default);
                                        step.drums.kick = true;
                                    }
                                    let _ = DRUM_CHANNEL.try_send(DrumSample::Kick);
                                }
                                SNARE_DRUM_ID => {
                                    if sequencer_mode == SequencerMode::Record {
                                        // (修改) 使用 target_step
                                        let step = sequence[target_step]
                                            .get_or_insert_with(StepData::default);
                                        step.drums.snare = true;
                                    }
                                    let _ = DRUM_CHANNEL.try_send(DrumSample::Snare);
                                }
                                HAT_DRUM_ID => {
                                    if sequencer_mode == SequencerMode::Record {
                                        let step = sequence[target_step]
                                            .get_or_insert_with(StepData::default);
                                        step.drums.hat = true;
                                    }
                                    let _ = DRUM_CHANNEL.try_send(DrumSample::Hat);
                                }
                                USER_ID => {
                                    let sample_len =
                                        crate::USER_SAMPLE_READY.load(Ordering::SeqCst);

                                    if sample_len > 0 {
                                        // 2. 如果在录音模式，将触发写入步进
                                        if sequencer_mode == SequencerMode::Record {
                                            // 使用 target_step (如果你实现了量化) 或 current_step
                                            // 这里假设你用了 target_step 变量
                                            let step = sequence[current_step]
                                                .get_or_insert_with(StepData::default);

                                            step.drums.sample = true; // <-- 标记这一步播放采样
                                        }

                                        // 3. 实时触发 (让你听到你按下的声音)
                                        let _ = DRUM_CHANNEL.try_send(DrumSample::User);
                                    }
                                }
                                RECORD_START_ID => {
                                    let is_currently_recording =
                                        IS_RECORDING.load(Ordering::SeqCst);

                                    // 切换状态
                                    let new_state = !is_currently_recording;
                                    IS_RECORDING.store(new_state, Ordering::SeqCst);

                                    // 发送命令给 record_task (True=Start, False=Stop)
                                    let _ = RECORD_COMMAND_CHANNEL.try_send(new_state);
                                }
                                REVERSE_TOGGLE_ID => {
                                    // 3. 直接修改，不需要 unsafe，因为它是这个任务私有的
                                    is_reverse = !is_reverse;

                                    // 4. 发送状态给 audio_task
                                    let _ = REVERSE_STATE_CHANNEL.try_send(is_reverse);
                                }
                                _ => {}
                            },

                            ControlState::Dsp => match key_code {
                                CARRIER_WAVE_ID => {
                                    // Key 0
                                    wave_params.carrier_wave = match wave_params.carrier_wave {
                                        Waveform::Sine => Waveform::Triangle,
                                        Waveform::Triangle => Waveform::Sawtooth,
                                        Waveform::Sawtooth => Waveform::Square,
                                        Waveform::Square => Waveform::Sine,
                                    };
                                    let _ = WAVE_PARAMS_CHANNEL.try_send(wave_params);
                                }
                                MOD_WAVE_ID => {
                                    // Key 1
                                    wave_params.mod_wave = match wave_params.mod_wave {
                                        Waveform::Sine => Waveform::Triangle,
                                        Waveform::Triangle => Waveform::Sawtooth,
                                        Waveform::Sawtooth => Waveform::Square,
                                        Waveform::Square => Waveform::Sine,
                                    };
                                    let _ = WAVE_PARAMS_CHANNEL.try_send(wave_params);
                                }
                                ENV_TOGGLE_ID => {
                                    // Key 2: 切换包络焦点 (Amp <-> FM)
                                    active_env = match active_env {
                                        ActiveEnv::Amp => ActiveEnv::Fm,
                                        ActiveEnv::Fm => ActiveEnv::Amp,
                                    };
                                }
                                HAAS_TOGGLE_ID => {
                                    // Key 3: Haas Toggle
                                    is_haas_active = !is_haas_active;
                                    let _ = HAAS_STATE_CHANNEL.try_send(is_haas_active);
                                }
                                A_PARAM_ID => active_env_param = EnvParam::Attack, // Key 4
                                D_PARAM_ID => active_env_param = EnvParam::Decay,  // Key 5
                                S_PARAM_ID => active_env_param = EnvParam::Sustain, // Key 6
                                R_PARAM_ID => active_env_param = EnvParam::Release, // Key 7
                                MASTER_DRIVE_ID => active_env_param = EnvParam::MasterDrive,
                                BITCRUSER_ID => active_env_param = EnvParam::Bitcrusher,

                                _ => {}
                            },
                            _ => {}
                        }
                    }

                    _ => {}
                }
            } else if key_released {
                if key_code <= 12 && last_key_state[i] {
                    if note_keys_pressed > 0 {
                        note_keys_pressed -= 1;
                    }

                    if note_keys_pressed == 0 {
                        amp_envelope.note_off();
                        fm_envelope.note_off();
                    }
                }
            }
        }
        last_key_state = current_key_state;

        let now = Instant::now();
 if (sequencer_mode != SequencerMode::Stop)
            && (now.duration_since(last_tick_time) >= step_duration)
        {
            last_tick_time = now;
            current_step = (current_step + 1) & 31;
            if current_step & 3 == 0 {
                led.toggle();
            }
            
            // 1. 检查是否是刚刚手动录入的步进 (防双击)
            let is_skipped_step = skip_sequencer_step == Some(current_step);
            if is_skipped_step {
                skip_sequencer_step = None; // 重置标记
            }
            
            if !is_skipped_step {
                // --- (新!) 核心逻辑：数据劫持 ---
                // 优先使用 Stutter 锁定的数据；如果没有，才使用当前步进的数据
                let active_step_data = if stutter_locked_data.is_some() {
                    stutter_locked_data // 劫持！使用锁定的数据 (Beat Repeat)
                } else {
                    sequence[current_step] // 正常播放
                };

                // 统一处理播放逻辑 (无论是正常的还是劫持的)
                if let Some(step_data) = active_step_data {
                    
                    // A. 播放音符 (FM Synth)
                    if let Some(note) = step_data.note {
                        let base_frequency = NOTE_FREQUENCIES[note.key_code as usize]; 
                        let final_freq =
                            calculate_final_frequency(base_frequency, note.semitone, note.octave);

                        // 在 Stutter 模式下，我们也需要重新触发包络
                        fm_envelope.note_off(); // 确保瞬态
                        fm_envelope.note_on(); 
                        amp_envelope.note_off(); 
                        amp_envelope.note_on(); 
                        
                        let _ = AUDIO_CHANNEL.try_send(AudioCommand::Play(final_freq)); 
                    } else if note_keys_pressed == 0 {
                        // (这一步有鼓，但没有音符)
                        amp_envelope.note_off(); 
                        fm_envelope.note_off(); 
                    }

                    // B. 播放鼓组 (Drums & Samples)
                    if step_data.drums.kick {
                        let _ = DRUM_CHANNEL.try_send(DrumSample::Kick);
                    }
                    if step_data.drums.snare {
                        let _ = DRUM_CHANNEL.try_send(DrumSample::Snare);
                    }
                    if step_data.drums.hat {
                        let _ = DRUM_CHANNEL.try_send(DrumSample::Hat);
                    }
                    if step_data.drums.sample {
                        // 播放采样前检查长度
                        let sample_len = crate::USER_SAMPLE_READY.load(Ordering::SeqCst);
                        if sample_len > 0 {
                            let _ = DRUM_CHANNEL.try_send(DrumSample::User); 
                        }
                    }
                    
                } else if note_keys_pressed == 0 {
                    // (这一步是 None，完全是空的)
                    amp_envelope.note_off(); 
                    fm_envelope.note_off(); 
                }
            }
        }
        // --- 8E. 运行包络 & 发送参数 (已修改) ---
        let fm_env_val = fm_envelope.tick();
        let amp_env_val = amp_envelope.tick();

        // (修改) 我们只更新 index，Ratio 由编码器更新
        current_params.index = fm_env_val * max_fm_index;

        let _ = FM_PARAM_CHANNEL.try_send(current_params);
        let _ = AMP_CHANNEL.try_send(amp_env_val);

        // 8F. 停止播放
        if amp_envelope.is_idle() && note_keys_pressed == 0 && sequencer_mode == SequencerMode::Stop
        {
            let _ = AUDIO_CHANNEL.try_send(AudioCommand::Stop);
        }

        // --- 9. Await 5ms ---
        ticker.next().await;
    }
}

// #[task]
// pub async fn adc_task(mut adc: Adc<'static, ADC1>, mut pin: Peri<'static, PB0>) {
//     info!("ADC task (P15) started!");
//     loop {
//         let value = adc.blocking_read(&mut pin);
//         let _ = POT1_CHANNEL.try_send(value);
//         Timer::after_millis(20).await;
//     }
// }

// (encoder_task 已修复)
#[task]
pub async fn encoder_task(
    mut clk_a: ExtiInput<'static>,
    mut dt_b: ExtiInput<'static>,
    mut sw: ExtiInput<'static>,
) {
    info!("Encoder task (P7) started!");

    // 开关去抖参数（保持不变）
    let mut sw_stable_state = true;
    let mut sw_last_reading = true;
    let mut sw_stable_counter = 0;
    const DEBOUNCE_TICKS: u8 = 4; // 4 * 5ms = 20ms

    // 编码器状态跟踪
    let mut last_state = 0u8;
    let clk_init = clk_a.is_high() as u8;
    let dt_init = dt_b.is_high() as u8;
    last_state = (dt_init << 1) | clk_init; // DT (bit 1) | CLK (bit 0)

    // --- (新!) 旋转锁定状态 ---
    // 0: 等待变化; 1, 2, 3: 步进中; 4: 锁定/完成
    let mut rotation_lock_counter: u8 = 0;

    // (删除 Rotary Cooldown，因为我们使用状态锁定代替)

    loop {
        // 等待CLK或DT的任何边沿触发
        let clk_edge = clk_a.wait_for_any_edge();
        let dt_edge = dt_b.wait_for_any_edge();
        let sw_edge = sw.wait_for_any_edge();
        let tick = Timer::after_millis(5);

        match select(select(clk_edge, dt_edge), select(sw_edge, tick)).await {
            // --- 处理CLK或DT边沿变化（核心修改：增加状态锁定）---
            Either::First(_) => {
                // 1. 读取当前CLK和DT电平
                let clk_curr = clk_a.is_high() as u8;
                let dt_curr = dt_b.is_high() as u8;
                let curr_state = (dt_curr << 1) | clk_curr;

                // 2. 判断方向 (使用状态序列对比)
                let direction = match (last_state, curr_state) {
                    (0b00, 0b01) | (0b01, 0b11) | (0b11, 0b10) | (0b10, 0b00) => -1, // 逆时针
                    (0b00, 0b10) | (0b10, 0b11) | (0b11, 0b01) | (0b01, 0b00) => 1,  // 顺时针
                    _ => 0, // 无效状态（抖动或快速变化）
                };

                // 3. 核心锁定逻辑
                if direction != 0 {
                    // 如果是有效步进，增加计数器
                    rotation_lock_counter += 1;

                    // 只有当计数器达到 4 (完成一个完整四步) 时，才发送输出并重置
                    if rotation_lock_counter == 4 {
                        let _ = ENCODER_ROTARY_CHANNEL.try_send(direction);
                        rotation_lock_counter = 0; // 重置，等待下一个完整的 4 步
                    }
                } else if curr_state == last_state {
                    // 如果状态不变（物理触点静止），不应该影响计数器
                } else {
                    // 如果是无效状态（抖动，例如 00 -> 11），我们不发送，也不增加计数器
                }

                // 4. 更新上一次状态
                last_state = curr_state;
            }

            // --- 开关处理（保持不变）---
            Either::Second(either_sw_or_tick) => match either_sw_or_tick {
                // ... (开关去抖逻辑保持不变) ...
                Either::First(_) => {
                    sw_stable_counter = 0;
                }
                Either::Second(_) => {
                    let sw_curr_reading = sw.is_high();
                    if sw_curr_reading == sw_last_reading {
                        if sw_stable_counter < DEBOUNCE_TICKS {
                            sw_stable_counter += 1;
                            if sw_stable_counter == DEBOUNCE_TICKS {
                                if sw_curr_reading != sw_stable_state {
                                    sw_stable_state = sw_curr_reading;
                                    let pressed = !sw_stable_state;
                                    let _ = ENCODER_SWITCH_CHANNEL.try_send(pressed);
                                }
                            }
                        }
                    } else {
                        sw_stable_counter = 0;
                    }
                    sw_last_reading = sw_curr_reading;
                }
            },
        }
    }
}

#[embassy_executor::task]
pub async fn record_task(
    mut adc: Adc<'static, ADC1>,
    mut pot_pin: Peri<'static, PB0>, // PB0: 电位器读取
    mut mic_pin: Peri<'static, PB1>, // PB1: 麦克风录音
    sample_buffer: &'static mut [i16],
    sample_ready_len: &'static AtomicUsize,
) {
    let max_len = sample_buffer.len();
    // 为电位器创建一个 20ms 的定时器
    let mut pot_ticker = Ticker::every(Duration::from_millis(20));

    loop {
        // --- 1. 等待: "录音命令" 或 "电位器定时器" ---
        match select(RECORD_COMMAND_CHANNEL.receive(), pot_ticker.next()).await {
            // --- 1A. 收到了录音命令 (START) ---
            Either::First(command) => {
                if command == true {
                    info!("Recording started...");
                    Timer::after_millis(200).await;
                    let _ = adc.blocking_read(&mut mic_pin);
                    Timer::after_micros(10).await;

                    let mut current_pos = 0;

                    while current_pos < max_len {
                        // 检查是否收到停止命令
                        if let Ok(false) = RECORD_COMMAND_CHANNEL.try_receive() {
                            info!("Recording stopped by command.");
                            break;
                        }

                        let raw_sample: u16 = adc.blocking_read(&mut mic_pin);

                        let centered = raw_sample as i32 - 2050;

                        let amplified = centered * 1;

                        // 3. 软削波 (防止溢出 i16 范围)
                        let final_sample = amplified.clamp(-32768, 32767) as i16;

                        // 4. 写入缓冲区
                        sample_buffer[current_pos] = final_sample;

                        current_pos += 1;
                        // 保持采样率 (约 10kHz)
                        Timer::after_micros(80).await;
                    }

                    let mut peak_amp: i32 = 0;
                    for i in 0..current_pos {
                        let sample = sample_buffer[i] as i32;
                        // 使用 i32 取绝对值防止 -32768 溢出
                        let abs_sample = sample.abs(); 
                        if abs_sample > peak_amp {
                            peak_amp = abs_sample;
                        }
                        
                        // 关键：每处理 2000 个点让出一次 CPU，防止音频卡顿
                        // 2000 个简单的整数比较非常快，不会导致 Overrun
                        if i % 2000 == 0 {
                            Timer::after_micros(1).await; 
                        }
                    }

                    // 2. 应用增益 (Apply Gain)
                    // 目标峰值：32000 (留一点余量，不要顶到 32767 的极限)
                    // 只有当信号太小或有效时才处理
                    if peak_amp > 100 && peak_amp < 32000 {
                        let scale_factor = 32000.0 / peak_amp as f32;
                        info!("Peak: {}, Scaling by: {}", peak_amp, scale_factor);

                        for i in 0..current_pos {
                            let raw = sample_buffer[i] as f32;
                            let scaled = raw * scale_factor;
                            sample_buffer[i] = scaled as i16;

                            // 这里涉及浮点乘法，比较耗时，所以我们更频繁地让出 CPU
                            // 每 1000 个点休息一次
                            if i % 1000 == 0 {
                                Timer::after_micros(1).await;
                            }
                        }
                    } else {
                        info!("Skipping normalization (Peak: {})", peak_amp);
                    }
                    // ---------------------------------------

                    // 录音结束，更新长度
                    sample_ready_len.store(current_pos, Ordering::SeqCst); //
                    info!("Recording finished. Samples: {}", current_pos);
                }
            }

            // --- 1B. 电位器定时器触发 (平时状态) ---
            Either::Second(_) => {
                // 只有在不录音的时候，才读取电位器
                let pot_value: u16 = adc.blocking_read(&mut pot_pin);
                // 尝试发送，如果满了就丢弃，防止阻塞
                let _ = POT1_CHANNEL.try_send(pot_value);
            }
        }
    }
}
