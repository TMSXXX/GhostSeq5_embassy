#![no_std]
#![no_main]

mod synth;
mod wavetable;
use crate::synth::{
    DRUM_CHANNEL, DrumSample, FM_PARAM_CHANNEL, FmParams, MASTER_DRIVE_CHANNEL, UiState,
};
use crate::wavetable::{
    HAT_SAMPLE_LEN, KICK_SAMPLE_LEN, SNARE_SAMPLE_LEN, WAVE_TABLE_SIZE, WaveParams, Waveform,
    get_hat_sample_table, get_kick_sample_table, get_sawtooth_table, get_sine_table,
    get_snare_sample_table, get_square_table, get_triangle_table,
};
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
    timer,
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

static AUDIO_CHANNEL: Channel<CriticalSectionRawMutex, AudioCommand, 4> = Channel::new();
static AMP_CHANNEL: Channel<CriticalSectionRawMutex, f32, 2> = Channel::new();
static WAVE_PARAMS_CHANNEL: Channel<CriticalSectionRawMutex, WaveParams, 4> = Channel::new();

// ... (Executors, Interrupts, main() ... 不变) ...
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: InterruptExecutor = InterruptExecutor::new();
#[embassy_stm32::interrupt]
unsafe fn TIM2() {
    EXECUTOR_HIGH.on_interrupt();
}
#[embassy_stm32::interrupt]
unsafe fn TIM3() {
    EXECUTOR_MED.on_interrupt();
}
#[embassy_stm32::interrupt]
unsafe fn TIM5() {
    EXECUTOR_LOW.on_interrupt();
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

    interrupt::TIM5.set_priority(Priority::P4);

    // 2. 启动 P4 执行器

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
    interrupt::TIM3.set_priority(Priority::P4);
    //let spawner_med = EXECUTOR_MED.start(interrupt::TIM3);
    //interrupt::TIM5.set_priority(Priority::P15);
    //let spawner_low = EXECUTOR_LOW.start(interrupt::TIM5);

    // (Spawners 不变, audio_task 已恢复)
    //spawner.spawn(oled_task(display)).unwrap();
    spawner_high.spawn(synth::adc_task(adc, p.PB0)).unwrap();
    spawner_high.spawn(synth::control_task(keys, led)).unwrap();
    spawner_high
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
        bpm: 120.0,
        active_env: synth::ActiveEnv::Amp,
        active_env_param: synth::EnvParam::Attack,
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
        let mut bpm_text: String<16> = String::new();
        // 使用 {:.1} 只显示一位小数，确保显示稳定
        core::write!(bpm_text, "BPM: {:.1}", ui_state.bpm).unwrap_or(());
        Text::with_baseline(&bpm_text, Point::new(0, 40), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        match display.flush() {
            Ok(_) => {}
            Err(e) => {
                // 如果 I2C 写入失败，打印错误并继续下一个循环
                error!("OLED I2C flush error during transition");
            }
        }

        ticker.next().await;
    }
}

// (在 main.rs 中)
// 粘贴并替换你现有的 audio_task

#[embassy_executor::task()]
async fn audio_task(mut i2s: i2s::I2S<'static, u16>) {
    info!("Audio task starting...");

    // --- 1. 状态变量 (已回退到 FM) ---
    let mut frequency = 100.0f32;
    let mut is_on = false;
    let mut carrier_phase: f32 = 0.0;
    let mut modulator_phase: f32 = 0.0;
    let mut amplitude: f32 = 0.0;
    let mut wave_params = WaveParams {
        carrier_wave: Waveform::Triangle,
        mod_wave: Waveform::Square,
    };
    let mut params = FmParams {
        index: 1.5,
        ratio: 2.0,
    };

    // --- (新!) 鼓组复音状态 ---
    // 我们不再使用 'current_drum' 和 'drum_pos'
    // 而是为每个鼓维护一个独立的播放位置
    let mut kick_pos: Option<usize> = None;
    let mut snare_pos: Option<usize> = None;
    let mut hat_pos: Option<usize> = None;

    // --- 2. 缓冲区, Haas, 常量, 波表 (不变) ---
    let audio_buffers = AUDIO_BUFFERS.init([[0u16; HALF_DMA_LEN]; 2]);
    let mut current_buffer_idx = 0;
    let haas_delay_line = HAAS_DELAY_LINE.init([0i16; HAAS_DELAY_SIZE]);
    let mut haas_write_ptr: usize = 0;
    let mut haas_active: bool = false;
    let mut master_drive = 1.;
    const TABLE_MASK: usize = WAVE_TABLE_SIZE - 1;
    const TABLE_SIZE_F32: f32 = WAVE_TABLE_SIZE as f32;
    const TWO_PI: f32 = 2.0 * PI;
    const TWO_PI_INV: f32 = 0.15915494;
    let sine_table = get_sine_table();
    let sawtooth_table = get_sawtooth_table();
    let square_table = get_square_table();
    let triangle_table = get_triangle_table();

    let kick_samples = get_kick_sample_table();
    let snare_samples = get_snare_sample_table();
    let hat_samples = get_hat_sample_table();

    // --- 3. fill_buffer (已修改为支持鼓组复音) ---
    let mut fill_buffer = |buffer: &mut [u16; HALF_DMA_LEN],
                           freq: f32,
                           p: &FmParams,
                           wp: &WaveParams,
                           amp: f32, // 这是 FM 的 amp
                           cp: &mut f32,
                           mp: &mut f32,
                           // (新!) 传入所有鼓的状态
                           kick_pos: &mut Option<usize>,
                           snare_pos: &mut Option<usize>,
                           hat_pos: &mut Option<usize>,
                           on: bool,
                           hdl: &mut [i16; HAAS_DELAY_SIZE],
                           hwp: &mut usize,
                           haas_on: bool,
                           master_drive_val: f32| {
        let carrier_freq = freq;
        let modulator_freq = carrier_freq * p.ratio;
        let carrier_phase_increment = (TWO_PI * carrier_freq) / 48000.0;
        let modulator_phase_increment = (TWO_PI * modulator_freq) / 48000.0;

        for i in 0..SAMPLES_PER_BUFFER {
            // --- A. FM 合成 (不变) ---
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

                // (载波 A)
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

            // --- B. 鼓采样 (已修改为复音) ---
            let mut drum_sample_f32 = 0.0;

            // 1. 处理 Kick
            if let Some(pos) = kick_pos {
                if *pos < KICK_SAMPLE_LEN {
                    drum_sample_f32 += kick_samples[*pos]; // 混合
                    *kick_pos = Some(*pos + 1); // 推进位置
                } else {
                    *kick_pos = None; // 播放完毕
                }
            }

            // 2. 处理 Snare
            if let Some(pos) = snare_pos {
                if *pos < SNARE_SAMPLE_LEN {
                    drum_sample_f32 += snare_samples[*pos]; // 混合
                    *snare_pos = Some(*pos + 1); // 推进位置
                } else {
                    *snare_pos = None; // 播放完毕
                }
            }

            // 3. 处理 Hat
            if let Some(pos) = hat_pos {
                if *pos < HAT_SAMPLE_LEN {
                    drum_sample_f32 += hat_samples[*pos]; // 混合
                    *hat_pos = Some(*pos + 1); // 推进位置
                } else {
                    *hat_pos = None; // 播放完毕
                }
            }
            let mixed_sample_f32 = (fm_sample_f32 * amp * 0.3) + (drum_sample_f32 * 0.3); //

            // 1. 驱动信号
            let driven_signal = mixed_sample_f32 * master_drive_val;

            // 2. 饱和处理
            let saturated_signal = cheap_saturator(driven_signal);

            let final_sample_f32 = (saturated_signal / master_drive_val) * 5.0;

            let mono_sample_i16 = (final_sample_f32 * 32767.0) as i16;

            // Haas 效果 (不变)
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

            // 相位推进 (不变)
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

    // --- 4. 预填充 (已修改) ---
    fill_buffer(
        &mut audio_buffers[0],
        frequency,
        &params,
        &wave_params,
        amplitude,
        &mut carrier_phase,
        &mut modulator_phase,
        // (新!) 传入鼓状态
        &mut kick_pos,
        &mut snare_pos,
        &mut hat_pos,
        is_on,
        haas_delay_line,
        &mut haas_write_ptr,
        haas_active,
        master_drive,
    );
    fill_buffer(
        &mut audio_buffers[1],
        frequency,
        &params,
        &wave_params,
        amplitude,
        &mut carrier_phase,
        &mut modulator_phase,
        // (新!) 传入鼓状态
        &mut kick_pos,
        &mut snare_pos,
        &mut hat_pos,
        is_on,
        haas_delay_line,
        &mut haas_write_ptr,
        haas_active,
        master_drive,
    );

    // --- 5. I2S 启动 (不变) ---
    i2s.start();
    info!("I2S started!");
    let mut write_future = i2s.write(&audio_buffers[current_buffer_idx]);

    // --- 6. Audio Loop ---
    loop {
        // I2S 错误处理 (不变)
        match write_future.await {
            Ok(_) => {}
            Err(e) => {
                error!("I2S write error: {:?}", e);
                audio_buffers[0].fill(0);
                audio_buffers[1].fill(0);
                carrier_phase = 0.0;
                modulator_phase = 0.0;
                current_buffer_idx = 0;
                i2s.clear();
                haas_delay_line.fill(0);
                haas_write_ptr = 0;

                // (新!) 重置鼓状态
                kick_pos = None;
                snare_pos = None;
                hat_pos = None;

                write_future = i2s.write(&audio_buffers[current_buffer_idx]);
                continue;
            }
        }

        // 缓冲区交换 (不变)
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

        // --- 非关键路径 ---

        if let Ok(new_params) = FM_PARAM_CHANNEL.try_receive() {
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
        if let Ok(drive) = MASTER_DRIVE_CHANNEL.try_receive() {
            master_drive = drive;
        }
        // (新!) 处理鼓组触发
        while let Ok(drum) = synth::DRUM_CHANNEL.try_receive() {
            // 不再是替换，而是独立触发
            match drum {
                DrumSample::Kick => kick_pos = Some(0), // 从 0 开始播放 Kick
                DrumSample::Snare => snare_pos = Some(0), // 从 0 开始播放 Snare
                DrumSample::Hat => hat_pos = Some(0),   // 从 0 开始播放 Hat
            }
        }

        // (AudioCommand 接收... 不变)
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

        // (新!) 填充下一个缓冲区
        fill_buffer(
            buf_to_fill,
            frequency,
            &params,
            &wave_params,
            amplitude,
            &mut carrier_phase,
            &mut modulator_phase,
            // (新!) 传入鼓状态
            &mut kick_pos,
            &mut snare_pos,
            &mut hat_pos,
            is_on,
            haas_delay_line,
            &mut haas_write_ptr,
            haas_active,
            master_drive,
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
