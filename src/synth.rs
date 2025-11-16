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
use embassy_stm32::peripherals::{ADC1, PB0}; // <-- 明确指定引脚
use embassy_stm32::peripherals::{PA8, PA9, PA10};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::String;

use crate::{
    AMP_CHANNEL, // <-- 修复：确保 AMP_CHANNEL 被导入
    AUDIO_CHANNEL,
    AudioCommand,
    WAVE_PARAMS_CHANNEL,
    WAVE_TABLE_SIZE,
    WaveParams,
    Waveform,
    get_sawtooth_table,
    get_sine_table,
    get_square_table,
    get_triangle_table, // <-- 修复：你之前漏掉了 get_triangle_table
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FmParams {
    pub index: f32, // “金属感” (0.0 - 10.0)
    pub ratio: f32, // “音色” (0.5 - 5.0)
}

const NOTE_FREQUENCIES: [f32; 8] = [
    261.63, // C4 (Do) - 按键 0
    293.66, // D4 (Re) - 按键 1
    329.63, // E4 (Mi) - 按键 2
    349.23, // F4 (Fa) - 按键 3
    392.00, // G4 (So) - 按键 4
    440.00, // A4 (La) - 按键 5
    493.88, // B4 (Ti) - 按键 6
    523.25, // C5 (Do) - 按键 7
];

const SEMITONE_MULTIPLIERS: [f32; 13] = [
    // -5,   -4,    -3,    -2,    -1,    0
    0.7491, 0.7937, 0.8409, 0.8909, 0.9439, 1.0,
    // +1,    +2,    +3,    +4,    +5,    +6,    +7
    1.0595, 1.1225, 1.1892, 1.2599, 1.3348, 1.4142, 1.4983,
];
const SEMITONE_SHIFT_OFFSET: i8 = 5; // (用于将 -5..+7 映射到 0..12)

pub type OledText = String<16>;
pub static FM_PARAM_CHANNEL: Channel<CriticalSectionRawMutex, FmParams, 2> = Channel::new();
pub static ENCODER_ROTARY_CHANNEL: Channel<CriticalSectionRawMutex, i8, 4> = Channel::new();
/// 按键通道 (bool: true=按下, false=松开)
pub static ENCODER_SWITCH_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
pub static POT1_CHANNEL: Channel<CriticalSectionRawMutex, u16, 4> = Channel::new();
pub static HAAS_STATE_CHANNEL: Channel<CriticalSectionRawMutex, bool, 2> = Channel::new();
pub static UI_DASHBOARD_CHANNEL: Channel<CriticalSectionRawMutex, UiState, 2> = Channel::new();

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UiState {
    pub octave: usize,
    pub semitone: i8,
    pub fm_index: f32, // 当前 FM 指数 (0.0 - 10.0)
    pub carrier_wave: Waveform,
    pub mod_wave: Waveform,
    pub is_shift_held: bool,
    pub is_haas_active: bool,
    pub mode: SequencerMode,
    pub step: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum SequencerMode {
    Stop,
    Play,
    Record,
}

// 存储在音序器每一步的数据
#[derive(Debug, Clone, Copy, PartialEq)]
struct NoteData {
    key_code: u8, // 0-7
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

    // 时间 (单位：控制节拍, 5ms)
    attack_ticks: f32,
    decay_ticks: f32,
    sustain_level: f32,
    release_ticks: f32,

    // 预先计算的步长 (修复性能的关键!)
    attack_step: f32,
    decay_step: f32,
    release_step: f32,
    release_start_value: f32,
}



// 在 src/synth.rs 中

impl Envelope {
    fn new() -> Self {
        Self {
            stage: EnvelopeStage::Idle,
            current_value: 0.0,
            attack_ticks: 2.0,  // 10ms
            decay_ticks: 100.0, // 500ms
            sustain_level: 0.0,
            release_ticks: 40.0, // 100ms
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
    info!("Control task (P7) started!"); // <-- 启动时打印一次 OK

    // --- 1. 键盘状态 ---
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
    
    // --- 3. FM / 波形 / FX 状态 (不变) ---
    let mut current_params = FmParams { index: 0.0, ratio: 2.0 };
    let mut wave_params = WaveParams { carrier_wave: Waveform::Triangle, mod_wave: Waveform::Square };
    let mut is_haas_active = false;
    let mut is_shift_held = false; 
    let mut pot1_raw: u16 = 2048;
    let mut max_fm_index: f32 = 2.0;

    // --- 4. 包络 (Envelopes) (不变) ---
    let mut fm_envelope = Envelope::new();
    fm_envelope.attack_ticks = 0.2; 
    fm_envelope.decay_ticks = 15.0; 
    fm_envelope.sustain_level = 0.0;
    fm_envelope.release_ticks = 6.0;
    let mut amp_envelope = Envelope::new();
    amp_envelope.attack_ticks = 0.2; 
    amp_envelope.decay_ticks = 40.0; 
    amp_envelope.sustain_level = 0.0;
    amp_envelope.release_ticks = 25.0;
    
    // --- 6. Sequencer (音序器) 状态 (不变) ---
    let mut sequencer_mode = SequencerMode::Stop;
    let mut current_step: usize = 0;
    let mut bpm: f32 = 120.0;
    let mut step_duration = Duration::from_millis(125); 
    let mut last_tick_time = Instant::now();
    let mut sequence: [Option<NoteData>; 16] = [None; 16]; 
    
    // --- 7. 功能键 ID (不变) ---
    const CARRIER_WAVE_ID: u8 = 11; 
    const MOD_WAVE_ID: u8 = 10;
    const HAAS_TOGGLE_ID: u8 = 12; 
    const PLAY_STOP_ID: u8 = 15; 
    const RECORD_ID: u8 = 14; 
    
    // (发送初始状态 ... 不变)
    let _ = FM_PARAM_CHANNEL.try_send(current_params);
    let _ = AMP_CHANNEL.try_send(0.0);
    let _ = WAVE_PARAMS_CHANNEL.try_send(wave_params); 
    let _ = HAAS_STATE_CHANNEL.try_send(is_haas_active);

    // -----------------------------------------------------------------
    // 8. P7 主循环 (使用 Ticker)
    // -----------------------------------------------------------------
    let mut ticker = Ticker::every(Duration::from_millis(5)); // <-- 核心修复 1

    loop {
        // --- 8A. 检查所有通道 (编码器/电位器) ---
        // (这部分 100% 不变)
        if let Ok(rotation) = crate::synth::ENCODER_ROTARY_CHANNEL.try_receive() {
            if is_shift_held {
                semitone_shift += { match rotation > 0 { true => 1, false => -1 } };
                semitone_shift = semitone_shift.clamp(-5, 7); 
            } else {
                if rotation > 0 { if octave_scale < 4 { octave_scale += 1; } } 
                else { if octave_scale > 0 { octave_scale -= 1; } }
            }
        }
        if let Ok(pressed) = crate::synth::ENCODER_SWITCH_CHANNEL.try_receive() {
            is_shift_held = pressed;
            if !is_shift_held { semitone_shift = 0; }
        }
        if let Ok(val) = crate::synth::POT1_CHANNEL.try_receive() {
            pot1_raw = val; 
            max_fm_index = (pot1_raw as f32 / 4096.0) * 10.0;
        }

        // --- 8B. 扫描键盘 (已修复) ---
        let mut current_key_state: [bool; 16] = [false; 16];
        for (r, row) in rows.iter_mut().enumerate() {
            row.set_low();

            for (c, col) in cols.iter().enumerate() {
                if col.is_low() { current_key_state[r * 4 + c] = true; }
            }
            row.set_high();
        }

        // --- 8C. 处理按键逻辑 (已升级) ---
        for i in 0..16 {
            let key_pressed = current_key_state[i] && !last_key_state[i];
            let key_released = !current_key_state[i] && last_key_state[i];
            let key_code = i as u8;

            if key_pressed {
                match key_code {
                    // --- 音符键 (0-7): 实时演奏/录制 ---
                    0..=7 => {
                        // info!("Note Key {} pressed", key_code); // <-- 核心修复 3: 移除日志
                        note_keys_pressed += 1;
                        let note_data = NoteData {
                            key_code,
                            octave: octave_scale,
                            semitone: semitone_shift,
                            is_sharp: is_sharp_active,
                        };
                        
                        if sequencer_mode == SequencerMode::Record {
                             sequence[current_step] = Some(note_data);
                             // info!("Rec Step {}: Note {}", current_step, key_code); // <-- 移除
                        }
                        
                        let base_frequency = NOTE_FREQUENCIES[key_code as usize];
                        current_frequency = calculate_final_frequency(
                            base_frequency, is_sharp_active, semitone_shift, octave_scale
                        );
                        
                        if note_keys_pressed == 1 {
                            fm_envelope.note_on();
                            amp_envelope.note_on(); 
                        }
                        
                        let _ = AUDIO_CHANNEL.try_send(AudioCommand::Play(current_frequency));
                    }
                    
                    // (其他功能键不变)
                    CARRIER_WAVE_ID => { 
                        wave_params.carrier_wave = match wave_params.carrier_wave {
                            Waveform::Sine => Waveform::Triangle,
                            Waveform::Triangle => Waveform::Sawtooth,
                            Waveform::Sawtooth => Waveform::Square,
                            Waveform::Square => Waveform::Sine,
                        };
                        let _ = WAVE_PARAMS_CHANNEL.try_send(wave_params);
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
                                // info!("Sequencer: PLAY"); // <-- 移除
                                last_tick_time = Instant::now(); 
                                current_step = 15; 
                                SequencerMode::Play
                            },
                            _ => { 
                                // info!("Sequencer: STOP"); // <-- 移除
                                let _ = AUDIO_CHANNEL.try_send(AudioCommand::Stop); 
                                SequencerMode::Stop
                            },
                        };
                    }
                    
                    RECORD_ID => { 
                        if is_shift_held {
                            // 1. 清空音序器
                            sequence = [None; 16];
                            // 2. 停止播放并重置
                            sequencer_mode = SequencerMode::Stop;
                            current_step = 0;
                            // 3. 停止所有声音
                            let _ = AUDIO_CHANNEL.try_send(AudioCommand::Stop);
                            amp_envelope.note_off();
                            fm_envelope.note_off();
                            
                            // (你可以在这里向 OLED 发送一个“已清除”的消息，
                            //  但现在我们只清除它)
                            
                        } else {
                            // (这是你之前的逻辑，保持不变)
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
                    // info!("Note Key {} released", key_code); // <-- 移除
                    if note_keys_pressed > 0 { note_keys_pressed -= 1; }
                    
                    // 修复“粘滞音符” (你上次的修改是正确的)
                    if note_keys_pressed == 0 {
                        amp_envelope.note_off(); 
                        fm_envelope.note_off();
                    }
                }
            }
        }
        last_key_state = current_key_state;
        
        // --- 8D. (新!) BPM 时钟 & 音序器播放 ---
        let now = Instant::now();
        if (sequencer_mode != SequencerMode::Stop) && (now.duration_since(last_tick_time) >= step_duration) {
            
            last_tick_time = now; 
            current_step = (current_step + 1) % 16; 
            // info!("SEQ Step: {}", current_step); // <-- 移除
            
            if let Some(note) = sequence[current_step] {
                let base_frequency = NOTE_FREQUENCIES[note.key_code as usize];
                let final_freq = calculate_final_frequency(
                    base_frequency, note.is_sharp, note.semitone, note.octave
                );
                
                fm_envelope.note_on();
                amp_envelope.note_on();
                let _ = AUDIO_CHANNEL.try_send(AudioCommand::Play(final_freq));
            
            } else if note_keys_pressed == 0 { 
                amp_envelope.note_off(); 
                fm_envelope.note_off();
            }
        }

        // --- 8E. 运行包络 & 发送参数 (不变) ---
        let fm_env_val = fm_envelope.tick();
        let amp_env_val = amp_envelope.tick();
        current_params.index = fm_env_val * max_fm_index; 
        let _ = FM_PARAM_CHANNEL.try_send(current_params); 
        let _ = AMP_CHANNEL.try_send(amp_env_val); 

        // --- 8F. (不变) 停止播放逻辑 ---
        if amp_envelope.is_idle() && note_keys_pressed == 0 && sequencer_mode == SequencerMode::Stop {
             let _ = AUDIO_CHANNEL.try_send(AudioCommand::Stop);
        }

        // --- 8G. (新!) 发送 UI 状态包 ---
        // (这部分不变，它只是 try_send，非常快)
        let ui_state = UiState {
            octave: octave_scale,
            semitone: semitone_shift,
            fm_index: max_fm_index,
            carrier_wave: wave_params.carrier_wave,
            mod_wave: wave_params.mod_wave,
            is_shift_held: is_shift_held,
            is_haas_active: is_haas_active,
            mode: sequencer_mode,
            step: current_step,
        };
        // 这个 channel 必须被 OLED 任务消费，否则它也会满
        // 但 OLED 任务饿死了，所以 channel 满了，try_send 失败 (但不会阻塞)
        let _ = UI_DASHBOARD_CHANNEL.try_send(ui_state);

        // --- 9. Await 5ms ---
        // Timer::after_millis(5).await; // <-- (旧的)
        ticker.next().await; // <-- (新的) 确保 5ms 周期
    }
}



#[task]
pub async fn adc_task(mut adc: Adc<'static, ADC1>, mut pin: Peri<'static, PB0>) {
    info!("ADC task (P15) started!");
    loop {
        // 读取 ADC 值 (0-4095)
        let value = adc.blocking_read(&mut pin);

        let _ = POT1_CHANNEL.try_send(value);

        // 以 50Hz (20ms) 的速率轮询
        Timer::after_millis(20).await;
    }
}

#[task]
pub async fn encoder_task(
    mut clk_a: ExtiInput<'static>, // A 引脚 (CLK/PA8)
    mut dt_b: ExtiInput<'static>,  // B 引脚 (DT/PA9)
    mut sw: ExtiInput<'static>,    // SW 引脚 (按键/PA10)
) {
    info!("Encoder task (P7) started!");

    // 状态变量：记录按键的上一状态（初始化为当前状态，通常上拉为高电平）
    let mut sw_prev_state = sw.is_high();

    loop {
        let clk_fall = clk_a.wait_for_falling_edge();
        let sw_any_edge = sw.wait_for_any_edge(); // 等待任意边沿，但不依赖 Edge 枚举

        match select(clk_fall, sw_any_edge).await {
            // 旋转
            Either::First(_) => {
                if dt_b.is_low() {
                    let _ = ENCODER_ROTARY_CHANNEL.try_send(1); // 顺时针
                } else {
                    let _ = ENCODER_ROTARY_CHANNEL.try_send(-1); // 逆时针
                }
            }

            // 按键
            Either::Second(_) => {
                // 消抖：等待 20ms 确认状态稳定
                Timer::after_millis(20).await;

                // 读取当前状态
                let sw_curr_state = sw.is_high();

                // 比较上一状态和当前状态，判断是下降沿还是上升沿
                if sw_prev_state && !sw_curr_state {
                    // 上一状态高，当前状态低 → 下降沿（按下）
                    let _ = ENCODER_SWITCH_CHANNEL.try_send(true);
                } else if !sw_prev_state && sw_curr_state {
                    // 上一状态低，当前状态高 → 上升沿（松开）
                    let _ = ENCODER_SWITCH_CHANNEL.try_send(false);
                }

                // 更新状态变量（保存当前状态为下一次的“上一状态”）
                sw_prev_state = sw_curr_state;
            }
        }
    }
}
