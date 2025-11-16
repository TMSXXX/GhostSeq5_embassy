#![no_std]
#![no_main]

mod synth;
// (已回退) 导入 FM_PARAM_CHANNEL, FmParams
use crate::synth::{DRUM_CHANNEL, DrumSample, FM_PARAM_CHANNEL, FmParams, UiState};
use core::fmt::Write;
use cortex_m::peripheral::SCB;
use defmt::*;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_futures::select::{Either, select};
use embassy_stm32::{
    Peri,
    adc::{Adc, SampleTime},
    bind_interrupts,
    exti::ExtiInput,
    gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed},
    i2c::{self, I2c},
    i2s,
    interrupt::{self, InterruptExt, Priority},
    peripherals,
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
use heapless::String;
use micromath::F32Ext;
use once_cell::sync::OnceCell;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

use core::f32::consts::PI;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use embedded_graphics::{
    mono_font::MonoTextStyle,
    primitives::{Line, PrimitiveStyle},
};
use embedded_graphics::{mono_font::ascii::FONT_6X10, text::Baseline};
use embedded_graphics::{pixelcolor::BinaryColor, primitives::Rectangle};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

// ... (OledDisplay, OledText, 常量, 波表... 不变) ...
type OledDisplay = Ssd1306<
    I2CInterface<I2c<'static, embassy_stm32::mode::Async, embassy_stm32::i2c::Master>>,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
>;
type OledText = String<16>;
const AUDIO_DMA_BUF_SIZE: usize = 2400;
static DMA_BUF_CELL: StaticCell<[u16; AUDIO_DMA_BUF_SIZE]> = StaticCell::new();
const HALF_DMA_LEN: usize = AUDIO_DMA_BUF_SIZE / 2; // 1200
const SAMPLES_PER_BUFFER: usize = HALF_DMA_LEN / 2; // 600
static AUDIO_BUFFERS: StaticCell<[[u16; HALF_DMA_LEN]; 2]> = StaticCell::new();
const HAAS_DELAY_MS: usize = 20;
const HAAS_DELAY_SIZE: usize = (48000 * HAAS_DELAY_MS) / 1000; // 960
static HAAS_DELAY_LINE: StaticCell<[i16; HAAS_DELAY_SIZE]> = StaticCell::new();

const WAVE_TABLE_SIZE: usize = 1024;
static SINE_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();
static SQUARE_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();
static SAWTOOTH_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();
static TRIANGLE_TABLE: OnceCell<[f32; WAVE_TABLE_SIZE]> = OnceCell::new();

const KICK_SAMPLE_LEN: usize = 2048;
static KICK_SAMPLE_TABLE: OnceCell<[f32; KICK_SAMPLE_LEN]> = OnceCell::new();
const SNARE_SAMPLE_LEN: usize = 1500;
static SNARE_SAMPLE_TABLE: OnceCell<[f32; SNARE_SAMPLE_LEN]> = OnceCell::new();
const HAT_SAMPLE_LEN: usize = 1000;
static HAT_SAMPLE_TABLE: OnceCell<[f32; HAT_SAMPLE_LEN]> = OnceCell::new();

fn get_sine_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    SINE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        for i in 0..WAVE_TABLE_SIZE {
            let phase = (i as f32 / WAVE_TABLE_SIZE as f32) * (2.0 * PI);
            table[i] = phase.sin();
        }
        info!("Sine table (1024 samples) generated.");
        table
    })
}
fn get_square_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    SQUARE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        for i in 0..WAVE_TABLE_SIZE {
            let phase = (i as f32 / WAVE_TABLE_SIZE as f32) * (2.0 * PI);
            table[i] = if phase < PI { 1.0 } else { -1.0 };
        }
        info!("Square table (1024 samples) generated.");
        table
    })
}
fn get_sawtooth_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    SAWTOOTH_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        for i in 0..WAVE_TABLE_SIZE {
            let normalized = i as f32 / WAVE_TABLE_SIZE as f32;
            table[i] = 2.0 * normalized - 1.0;
        }
        info!("Sawtooth table (1024 samples) generated.");
        table
    })
}
fn get_triangle_table() -> &'static [f32; WAVE_TABLE_SIZE] {
    TRIANGLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; WAVE_TABLE_SIZE];
        let half = WAVE_TABLE_SIZE / 2;
        for i in 0..WAVE_TABLE_SIZE {
            if i < half {
                let normalized = i as f32 / half as f32;
                table[i] = 2.0 * normalized - 1.0;
            } else {
                let normalized = (i - half) as f32 / half as f32;
                table[i] = 1.0 - 2.0 * normalized;
            }
        }
        info!("Triangle table (1024 samples) generated.");
        table
    })
}

fn get_kick_sample_table() -> &'static [f32; KICK_SAMPLE_LEN] {
    KICK_SAMPLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; KICK_SAMPLE_LEN];
        let mut freq = 200.0; // 起始频率
        let mut phase = 0.0;

        for i in 0..KICK_SAMPLE_LEN {
            // 1. 振荡器 (Sine)
            let sample = phase.sin();

            let drive = 3.0; // 提升 3 倍增益
            let distorted_sample = cheap_saturator(sample * drive);

            // 2. 音高包络 (非常快地下降)
            freq *= 0.999; // 频率指数衰减
            let phase_inc = (2.0 * PI * freq) / 48000.0;
            phase += phase_inc;
            if phase > (2.0 * PI) {
                phase -= 2.0 * PI;
            }

            // 3. 音量包络 (线性衰减)
            let amp_env = 1.0 - (i as f32 / KICK_SAMPLE_LEN as f32);

            table[i] = distorted_sample * amp_env;
        }
        info!("Kick sample table (2048 samples) generated.");
        table
    })
}
fn get_snare_sample_table() -> &'static [f32; SNARE_SAMPLE_LEN] {
    SNARE_SAMPLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; SNARE_SAMPLE_LEN];
        let mut tone_freq = 250.0; // 军鼓基础音高（高于底鼓）
        let mut phase = 0.0;
        let mut noise_seed = 42u32; // 用于生成伪随机噪音的种子

        for i in 0..SNARE_SAMPLE_LEN {
            // 1. 低频音调成分（正弦波，快速降调）
            let tone = phase.sin();
            tone_freq *= 0.997; // 音高衰减速度快于底鼓
            let phase_inc = (2.0 * PI * tone_freq) / 48000.0;
            phase += phase_inc;
            if phase > 2.0 * PI {
                phase -= 2.0 * PI;
            }

            // 2. 噪音成分（白噪音，提供"沙沙声"）
            // 用线性同余发生器生成伪随机数（嵌入式环境无标准随机库）
            noise_seed = noise_seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let noise = ((noise_seed as f32 / u32::MAX as f32) * 2.0) - 1.0; // 映射到[-1,1]

            // 3. 噪音与音调混合（随时间变化比例）
            let noise_ratio = 0.7 + (i as f32 / SNARE_SAMPLE_LEN as f32) * 0.3; // 后期噪音占比提高
            let mixed = (tone * (1.0 - noise_ratio)) + (noise * noise_ratio);

            // 4. 失真处理（增强金属感）
            let drive = 2.5;
            let distorted = cheap_saturator(mixed * drive);

            // 5. 音量包络（前10%快速上升，后90%快速衰减）
            let env_attack = (i as f32 / (SNARE_SAMPLE_LEN as f32 * 0.1)).min(1.0);
            let env_decay = 1.0 - (i as f32 / SNARE_SAMPLE_LEN as f32);
            let amp_env = env_attack * env_decay;

            table[i] = distorted * amp_env * 0.8; // 降低整体增益避免削波
        }
        info!("Snare sample table (1500 samples) generated.");
        table
    })
}
fn get_hat_sample_table() -> &'static [f32; HAT_SAMPLE_LEN] {
    HAT_SAMPLE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; HAT_SAMPLE_LEN];
        let mut noise_seed = 123u32;
        
        for i in 0..HAT_SAMPLE_LEN {
            // 高频噪音生成
            noise_seed = noise_seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let noise = ((noise_seed as f32 / u32::MAX as f32) * 2.0) - 1.0;
            
            // 快速衰减包络
            let env = if i < HAT_SAMPLE_LEN / 20 {
                (i as f32 * 20.0) / HAT_SAMPLE_LEN as f32
            } else {
                (1.0 - (i as f32 / HAT_SAMPLE_LEN as f32)).powf(2.0)
            };
            
            table[i] = noise * env * 0.5;
        }
        info!("Hat sample table generated.");
        table
    })
}

// ... (Enums, Structs ... 已回退) ...
#[derive(Debug, Clone, Copy)]
enum Keyboard {
    KeyPress(u8),
    KeyRelease(u8),
}
#[derive(Debug, Clone, Copy)]
enum AudioCommand {
    Play(f32),
    Stop,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Waveform {
    Sine,
    Triangle,
    Sawtooth,
    Square,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaveParams {
    pub carrier_wave: Waveform,
    pub mod_wave: Waveform,
}
// (FilterState 已移除)

static AUDIO_CHANNEL: Channel<CriticalSectionRawMutex, AudioCommand, 4> = Channel::new();
static AMP_CHANNEL: Channel<CriticalSectionRawMutex, f32, 2> = Channel::new();
static WAVE_PARAMS_CHANNEL: Channel<CriticalSectionRawMutex, WaveParams, 4> = Channel::new();

// ... (Executors, Interrupts, main() ... 不变) ...
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();
#[embassy_stm32::interrupt]
unsafe fn TIM2() {
    EXECUTOR_HIGH.on_interrupt();
}
#[embassy_stm32::interrupt]
unsafe fn TIM3() {
    EXECUTOR_MED.on_interrupt();
}
bind_interrupts!(
    struct Irqs {
        I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    }
);
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // (FPU, Config, Init ... 不变)
    unsafe {
        let scb = SCB::ptr();
        (*scb).shpr[11].write(Priority::P6.into());
    }
    enable_fpu();
    let config = {
        //
        use embassy_stm32::rcc::*;
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(25),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL192,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV4),
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.plli2s = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL384,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV5),
        });
        config.enable_debug_during_sleep = true;
        config
    };
    let p = embassy_stm32::init(config);
    info!("System starting...");
    Timer::after_millis(100).await;
    let mut i2c_config = embassy_stm32::i2c::Config::default();
    i2c_config.frequency = Hertz(400_000);
    i2c_config.sda_pullup = true;
    i2c_config.scl_pullup = true;
    let i2c = I2c::new(
        p.I2C1, p.PB8, p.PB9, Irqs, p.DMA1_CH1, p.DMA1_CH0, i2c_config,
    );
    info!("I2C initialized.");
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    info!("OLED display created.");
    let keys: [[Peri<'static, AnyPin>; 4]; 2] = [
        [p.PA3.into(), p.PA2.into(), p.PA1.into(), p.PA0.into()],
        [p.PA7.into(), p.PA6.into(), p.PA5.into(), p.PA4.into()],
    ];
    info!("Keyboard pins configured.");
    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES15);
    let mut i2s_config = i2s::Config::default();
    i2s_config.format = i2s::Format::Data16Channel32;
    i2s_config.master_clock = false;
    i2s_config.frequency = Hertz(48000);
    info!("I2S config set.");

    // led配置
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);
    
    let enc_a = ExtiInput::new(p.PA9, p.EXTI9, Pull::Up);
    let enc_b = ExtiInput::new(p.PA8, p.EXTI8, Pull::Up);
    let enc_sw = ExtiInput::new(p.PA10, p.EXTI10, Pull::Up);
    let i2s = i2s::I2S::new_txonly_nomck(
        p.SPI3,
        p.PB5,
        p.PA15,
        p.PB3,
        p.DMA1_CH7,
        DMA_BUF_CELL.init([0u16; AUDIO_DMA_BUF_SIZE]),
        i2s_config,
    );
    info!("I2S configured.");
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap();
    display.flush().unwrap();
    get_sine_table();
    get_sawtooth_table();
    get_square_table();
    get_triangle_table();
    get_kick_sample_table();
    interrupt::TIM2.set_priority(Priority::P3);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::TIM2);
    interrupt::TIM3.set_priority(Priority::P5);
    let spawner_med = EXECUTOR_MED.start(interrupt::TIM3);

    // (Spawners 不变, audio_task 已恢复)
    spawner.spawn(oled_task(display)).unwrap();
    spawner_med.spawn(synth::adc_task(adc, p.PB0)).unwrap();
    spawner_med.spawn(synth::control_task(keys, led)).unwrap();
    spawner_med
        .spawn(synth::encoder_task(enc_a, enc_b, enc_sw))
        .unwrap();
    spawner_high.spawn(audio_task(i2s)).unwrap();

    info!("All tasks started, system ready!");
}

// (enable_fpu 不变)
fn enable_fpu() {
    unsafe {
        let scb = cortex_m::peripheral::SCB::ptr();
        (*scb).cpacr.modify(|r| r | (0b1111 << 20));
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

// (oled_task 不变)
#[embassy_executor::task]
async fn oled_task(mut display: OledDisplay) {
    info!("OLED task started!");
    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let clear_style = PrimitiveStyle::with_fill(BinaryColor::Off);
    let mut frequency: f32 = 0.0;
    let mut key_text: String<16> = String::new();
    core::write!(key_text, "Init...").unwrap();
    let mut ui_state = synth::UiState {
        mode: synth::SequencerMode::Stop,
        step: 0,
        octave: 2,
        semitone: 0,
        fm_index: 0.0,
        carrier_wave: crate::Waveform::Sine,
        mod_wave: crate::Waveform::Sine,
        is_shift_held: false,
        is_haas_active: false,
    };
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap();
    display.flush().unwrap();
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_millis(100));
    loop {
        let mut last_state_received = None;
        while let Ok(state) = synth::UI_DASHBOARD_CHANNEL.try_receive() {
            last_state_received = Some(state);
        }
        if let Some(new_state) = last_state_received {
            ui_state = new_state;
        }
        Rectangle::new(Point::new(0, 0), Size::new(128, 64))
            .into_styled(clear_style)
            .draw(&mut display)
            .unwrap();
        let mut status_text: String<16> = String::new();
        core::write!(
            status_text,
            "OCT: {} ST: {}",
            ui_state.octave,
            ui_state.semitone
        )
        .unwrap_or(());
        Text::with_baseline(&status_text, Point::new(0, 10), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        let mut fm_text: String<16> = String::new();
        core::write!(fm_text, "IDX: {}", (ui_state.fm_index * 10.0) as i32).unwrap_or(());
        Text::with_baseline(&fm_text, Point::new(0, 20), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        let mut wave_text: String<16> = String::new();
        core::write!(
            wave_text,
            "C: {} M: {}",
            wave_to_short_str(ui_state.carrier_wave),
            wave_to_short_str(ui_state.mod_wave)
        )
        .unwrap_or(());
        Text::with_baseline(&wave_text, Point::new(0, 30), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        match display.flush() {
            Ok(_) => {},
            Err(e) => {
                // 如果 I2C 写入失败，打印错误并继续下一个循环
                error!("OLED I2C flush error during transition");
            }
        }
        
        ticker.next().await;
    }
}

#[embassy_executor::task()]
async fn audio_task(mut i2s: i2s::I2S<'static, u16>) {
    info!("Audio task starting...");

    // --- 状态变量 (已回退) ---
    let mut frequency = 100.0f32;
    let mut is_on = false;
    let mut carrier_phase: f32 = 0.0;
    let mut modulator_phase: f32 = 0.0; // (已恢复)
    let mut amplitude: f32 = 0.0;
    let mut wave_params = WaveParams {
        carrier_wave: Waveform::Triangle,
        mod_wave: Waveform::Square,
    };
    let mut current_drum: Option<DrumSample> = None; // 用Option<DrumSample>表示当前播放的鼓
    let mut drum_pos: usize = 0; // 通用采样位置指针
    // (已恢复)
    let mut params = FmParams {
        index: 1.5,
        ratio: 2.0,
    };
    // (Filter 已移除)

    // ... (Audio Buffers, Haas, 常量, 波表获取... 不变) ...
    let audio_buffers = AUDIO_BUFFERS.init([[0u16; HALF_DMA_LEN]; 2]);
    let mut current_buffer_idx = 0;
    let haas_delay_line = HAAS_DELAY_LINE.init([0i16; HAAS_DELAY_SIZE]);
    let mut haas_write_ptr: usize = 0;
    let mut haas_active: bool = false;
    const TABLE_MASK: usize = WAVE_TABLE_SIZE - 1;
    const TABLE_SIZE_F32: f32 = WAVE_TABLE_SIZE as f32;
    const TWO_PI: f32 = 2.0 * PI;
    const TWO_PI_INV: f32 = 0.15915494;
    let sine_table = SINE_TABLE.get().unwrap();
    let sawtooth_table = SAWTOOTH_TABLE.get().unwrap();
    let square_table = SQUARE_TABLE.get().unwrap();
    let triangle_table = TRIANGLE_TABLE.get().unwrap();

    let kick_samples = get_kick_sample_table();
    let snare_samples = get_snare_sample_table();
    let hat_samples = get_hat_sample_table();

    // --- fill_buffer (已回退到 FM 版本) ---
    let mut fill_buffer = |buffer: &mut [u16; HALF_DMA_LEN],
                           freq: f32,
                           p: &FmParams,
                           wp: &WaveParams,
                           amp: f32, // 这是 FM 的 amp
                           cp: &mut f32,
                           mp: &mut f32,
                           current_drum: &mut Option<DrumSample>,
                           drum_pos: &mut usize,
                           on: bool,
                           hdl: &mut [i16; HAAS_DELAY_SIZE],
                           hwp: &mut usize,
                           haas_on: bool| {
        // (已恢复 FM 逻辑)
        let carrier_freq = freq;
        let modulator_freq = carrier_freq * p.ratio;
        let carrier_phase_increment = (TWO_PI * carrier_freq) / 48000.0;
        let modulator_phase_increment = (TWO_PI * modulator_freq) / 48000.0;

        for i in 0..SAMPLES_PER_BUFFER {
            let fm_sample_f32 = if on {
                // (调制波 B)
                let mod_phase_rads = *mp;
                let mod_index_f32 = (mod_phase_rads * TWO_PI_INV) * TABLE_SIZE_F32;
                let mod_idx0 = (mod_index_f32 as i32) as usize & TABLE_MASK;

                let mod_val = match wp.mod_wave {
                    Waveform::Sine => sine_table[mod_idx0],
                    Waveform::Triangle => triangle_table[mod_idx0],
                    Waveform::Sawtooth => sawtooth_table[mod_idx0],
                    Waveform::Square => square_table[mod_idx0],
                };

                let phase_offset = mod_val * p.index;
                let mut carrier_phase_rads = *cp + phase_offset;
                while carrier_phase_rads < 0.0 {
                    carrier_phase_rads += TWO_PI;
                }

                // (载波 A - 无插值)
                let carrier_index_f32 = (carrier_phase_rads * TWO_PI_INV) * TABLE_SIZE_F32;
                let carrier_idx0 = (carrier_index_f32 as i32) as usize & TABLE_MASK;

                match wp.carrier_wave {
                    Waveform::Sine => sine_table[carrier_idx0],
                    Waveform::Triangle => triangle_table[carrier_idx0],
                    Waveform::Sawtooth => sawtooth_table[carrier_idx0],
                    Waveform::Square => square_table[carrier_idx0],
                }
            } else {
                0.0
            };

            // 鼓采样
            let mut drum_sample_f32 = 0.0;
            if let Some(drum_type) = current_drum {
                // 根据当前鼓类型获取对应的采样表和长度
                let (samples, max_len) = match drum_type {
                    DrumSample::Kick => (kick_samples.as_slice(), KICK_SAMPLE_LEN),
                    DrumSample::Snare => (snare_samples.as_slice(), SNARE_SAMPLE_LEN),
                    DrumSample::Hat => (hat_samples.as_slice(), HAT_SAMPLE_LEN),
                };

                if *drum_pos < max_len {
                    drum_sample_f32 = samples[*drum_pos];
                    *drum_pos += 1;
                } else {
                    // 采样播放结束，重置状态
                    *current_drum = None;
                    *drum_pos = 0;
                }
            }

            // --- 3. 放大器 (Amp) ---
            let mixed_sample_f32 = (fm_sample_f32 * amp * 0.7) + (drum_sample_f32 * 0.7);
            let mono_sample_i16 = (mixed_sample_f32 * 32767.0) as i16;

            // ... (Haas 和 相位推进 逻辑不变) ...
            let read_ptr = *hwp;
            let delayed_sample_i16 = hdl[read_ptr];
            hdl[read_ptr] = mono_sample_i16;
            *hwp += 1;
            if *hwp >= HAAS_DELAY_SIZE {
                *hwp = 0;
            }
            if haas_on {
                buffer[i * 2] = mono_sample_i16 as u16;
                buffer[i * 2 + 1] = delayed_sample_i16 as u16;
            } else {
                buffer[i * 2] = mono_sample_i16 as u16;
                buffer[i * 2 + 1] = mono_sample_i16 as u16;
            }

            *cp += carrier_phase_increment;
            *mp += modulator_phase_increment;
            if *cp > TWO_PI {
                *cp -= TWO_PI;
            }
            if *mp > TWO_PI {
                *mp -= TWO_PI;
            }
        }
        if !on {
            *cp = 0.0;
            *mp = 0.0;
        }
    }; // (fill_buffer 闭包结束)

    // --- 预填充 (已回退) ---
    fill_buffer(
        &mut audio_buffers[0],
        frequency,
        &params,
        &wave_params,
        amplitude,
        &mut carrier_phase,
        &mut modulator_phase,
        &mut current_drum,  // 传入Option<DrumSample>
        &mut drum_pos,      // 传入通用位置指针
        is_on,
        haas_delay_line,
        &mut haas_write_ptr,
        haas_active,
    );
    fill_buffer(
        &mut audio_buffers[1],
        frequency,
        &params,
        &wave_params,
        amplitude,
        &mut carrier_phase,
        &mut modulator_phase,
        &mut current_drum,  // 同上
        &mut drum_pos,
        is_on,
        haas_delay_line,
        &mut haas_write_ptr,
        haas_active,
    );

    // ... (i2s.start(), write_future 不变) ...
    i2s.start();
    info!("I2S started!");
    let mut write_future = i2s.write(&audio_buffers[current_buffer_idx]);

    loop {
       
        // ... (match write_future.await... 错误处理不变) ...
        match write_future.await {
            Ok(_) => {}
            Err(e) => {
                error!("I2S write error: {:?}", e);
                audio_buffers[0].fill(0);
                audio_buffers[1].fill(0);
                carrier_phase = 0.0;
                modulator_phase = 0.0; // (已恢复)
                current_buffer_idx = 0;
                i2s.clear();
                haas_delay_line.fill(0);
                haas_write_ptr = 0;
                write_future = i2s.write(&audio_buffers[current_buffer_idx]);
                continue;
            }
        }

        // --- 关键路径 (不变) ---
        // ... (缓冲区交换 不变) ...
        let (buf0_slice, buf1_slice) = audio_buffers.split_at_mut(1);
        let buf0 = &mut buf0_slice[0];
        let buf1 = &mut buf1_slice[0];
        current_buffer_idx ^= 1;
        let (buf_to_write, buf_to_fill);
        if current_buffer_idx == 0 {
            buf_to_write = buf0;
            buf_to_fill = buf1;
        } else {
            buf_to_write = buf1;
            buf_to_fill = buf0;
        }
        write_future = i2s.write(buf_to_write);
        // --- S 关键路径结束 ---

        // --- 非关键路径 (已回退) ---
        // (移除了 MOD_ENV 和 FILTER_PARAMS)
        if let Ok(new_params) = FM_PARAM_CHANNEL.try_receive() {
            // (已恢复)
            params = new_params;
        }
        if let Ok(new_amp) = AMP_CHANNEL.try_receive() {
            amplitude = new_amp;
        }
        if let Ok(haas_on) = synth::HAAS_STATE_CHANNEL.try_receive() {
            haas_active = haas_on;
        }
        if let Ok(new_waves) = WAVE_PARAMS_CHANNEL.try_receive() {
            wave_params = new_waves;
        }
         while let Ok(drum) = synth::DRUM_CHANNEL.try_receive() {
            // 直接用收到的DrumSample更新当前状态，中断现有播放
            current_drum = Some(drum);
            drum_pos = 0; // 重置指针
        }
        while let Ok(command) = AUDIO_CHANNEL.try_receive() {
            match command {
                AudioCommand::Play(freq) => {
                    frequency = freq;
                    is_on = true;
                }
                AudioCommand::Stop => {
                    is_on = false;
                }
            }
        }

        // (填充下一个缓冲区 已回退)
        fill_buffer(
            buf_to_fill,
            frequency,
            &params,
            &wave_params,
            amplitude,
            &mut carrier_phase,
            &mut modulator_phase,
            &mut current_drum, // 传入Option<DrumSample>
            &mut drum_pos,
            is_on,
            haas_delay_line,
            &mut haas_write_ptr,
            haas_active,
        );
    }
}

const fn wave_to_short_str(wave: Waveform) -> &'static str {
    match wave {
        Waveform::Sine => "SINE",
        Waveform::Triangle => "TRI",
        Waveform::Sawtooth => "SAW",
        Waveform::Square => "SQU",
    }
}

fn cheap_saturator(x: f32) -> f32 {
    x / (1.0 + x.abs())
}
