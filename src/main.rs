#![no_std]
#![no_main]

mod display;
mod dsp;
mod synth;
mod wavetable;
use crate::display::Max7219;
use crate::dsp::cheap_saturator;
use crate::synth::{
    BITCRUSH_CHANNEL, DRUM_CHANNEL, DrumSample, FM_PARAM_CHANNEL, FmParams, MASTER_DRIVE_CHANNEL, MASTER_VOLUME_CHANNEL, REVERSE_STATE_CHANNEL, SAMPLE_PITCH_CHANNEL, TAPE_FACTOR_CHANNEL
};
use crate::wavetable::{
    HAT_SAMPLE_LEN, KICK_SAMPLE_LEN, SNARE_SAMPLE_LEN, WAVE_TABLE_SIZE, WaveParams, Waveform,
    get_hat_sample_table, get_kick_sample_table, get_sawtooth_table, get_sine_table,
    get_snare_sample_table, get_square_table, get_triangle_table,
};
use cortex_m::peripheral::SCB;
use defmt::*;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_stm32::{
    Peri,
    adc::{Adc, SampleTime},
    exti::ExtiInput,
    gpio::{AnyPin, Input, Level, Output, Pin, Pull, Speed},
    i2s,
    interrupt::{self, InterruptExt, Priority},
    time::Hertz,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
use once_cell::sync::OnceCell;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

use core::f32::consts::PI;
use core::sync::atomic::{AtomicUsize, Ordering};

const AUDIO_DMA_BUF_SIZE: usize = 2400;
static DMA_BUF_CELL: StaticCell<[u16; AUDIO_DMA_BUF_SIZE]> = StaticCell::new();
const HALF_DMA_LEN: usize = AUDIO_DMA_BUF_SIZE / 2; // 1200
const SAMPLES_PER_BUFFER: usize = HALF_DMA_LEN / 2; // 600
static AUDIO_BUFFERS: StaticCell<[[u16; HALF_DMA_LEN]; 2]> = StaticCell::new();
const HAAS_DELAY_MS: usize = 20;
const HAAS_DELAY_SIZE: usize = (48000 * HAAS_DELAY_MS) / 1000; // 960
static HAAS_DELAY_LINE: StaticCell<[i16; HAAS_DELAY_SIZE]> = StaticCell::new();

pub const USER_SAMPLE_LEN: usize = 24000;

// --- (修改) 使用 static mut 直接分配内存 ---
// 这种方式由链接器直接在 RAM 中保留空间，启动时自动清零，不占用栈
static mut USER_SAMPLE_MEMORY: [i16; USER_SAMPLE_LEN] = [0; USER_SAMPLE_LEN];
// 2. 使用 AtomicUsize 存储长度（用于安全共享）
static USER_SAMPLE_READY: AtomicUsize = AtomicUsize::new(0);

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
// 控制录音：true=开始录音，false=停止录音/等待
static RECORD_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, bool, 1> = Channel::new();

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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // (FPU, Config, Init ... 不变)
    unsafe {
        let scb = SCB::ptr();
        (*scb).shpr[11].write(Priority::P6.into());
    }
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

    enable_fpu();
    interrupt::TIM5.set_priority(Priority::P4);

    // 2. 启动 P4 执行器

    info!("System starting...");
    Timer::after_millis(100).await;
    let keys: [[Peri<'static, AnyPin>; 4]; 2] = [
        [p.PA3.into(), p.PA2.into(), p.PA1.into(), p.PA0.into()],
        [p.PA7.into(), p.PA6.into(), p.PA5.into(), p.PA4.into()],
    ];
    info!("Keyboard pins configured.");
    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES480);
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
    let mut screen = Max7219::new(p.PB12.into(), p.PB13.into(), p.PB14.into());
    let ghost_frame_1: [u8; 8] = [0x18, 0x3C, 0x7E, 0xDB, 0xFF, 0xFF, 0xA5, 0x00];
    let ghost_frame_2: [u8; 8] = [
        0x18, 0x3C, 0x7E, 0xDB, 0xFF, 0xFF, 0x5A, 0x00, // 脚的位置变了
    ];

    info!("I2S configured.");
    get_sine_table();
    get_sawtooth_table();
    get_square_table();
    get_triangle_table();
    get_kick_sample_table();
    get_snare_sample_table();
    get_hat_sample_table();
    interrupt::TIM2.set_priority(Priority::P3);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::TIM2);
    interrupt::TIM3.set_priority(Priority::P4);

    // 2. 创建一个裸指针 (Raw Pointer)
    let raw_ptr = unsafe { (&raw mut USER_SAMPLE_MEMORY) as *mut i16 };
    let sample_ready_ref = &USER_SAMPLE_READY;

    let buffer_for_record = unsafe { core::slice::from_raw_parts_mut(raw_ptr, USER_SAMPLE_LEN) };

    let buffer_for_audio = unsafe { core::slice::from_raw_parts(raw_ptr, USER_SAMPLE_LEN) };
    //let spawner_med = EXECUTOR_MED.start(interrupt::TIM3);
    //interrupt::TIM5.set_priority(Priority::P15);
    //let spawner_low = EXECUTOR_LOW.start(interrupt::TIM5);
 
    //spawner.spawn(oled_task(display)).unwrap();

    spawner_high.spawn(synth::control_task(keys, led)).unwrap();
    spawner_high
        .spawn(synth::encoder_task(enc_a, enc_b, enc_sw))
        .unwrap();
    spawner_high
        .spawn(audio_task(
            i2s,
            buffer_for_audio,
            sample_ready_ref,
        ))
        .unwrap();

    spawner_high
        .spawn(synth::record_task(
            adc,
            p.PB0,
            p.PB1,
            buffer_for_record,
            sample_ready_ref,
        ))
        .unwrap();

    info!("All tasks started, system ready!");

    loop {

        screen.flush(&ghost_frame_1);
        Timer::after_millis(500).await; 

        screen.flush(&ghost_frame_2);
        Timer::after_millis(500).await;
    }
}

fn enable_fpu() {
    unsafe {
        let scb = cortex_m::peripheral::SCB::ptr();

        (*scb).cpacr.modify(|r| r | (0b1111 << 20));

        let fpccr = 0xE000EF34 as *mut u32;
        *fpccr = *fpccr | (1 << 31) | (1 << 30);

        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}


#[embassy_executor::task()]
async fn audio_task(
    mut i2s: i2s::I2S<'static, u16>,
    sample_buffer: &'static [i16],
    sample_ready_ref: &'static AtomicUsize,
) {
    info!("Audio task starting (F32 Safe Optimized)...");

    // 动态变量
    let mut frequency = 100.0f32;
    let mut is_on = false;
    let mut carrier_phase: f32 = 0.0;
    let mut modulator_phase: f32 = 0.0;
    let mut amplitude: f32 = 0.0;
    let mut playback_step: f32 = 0.18;
    let mut wave_params = WaveParams {
        carrier_wave: Waveform::Triangle,
        mod_wave: Waveform::Square,
    };
    let mut params = FmParams {
        index: 1.5,
        ratio: 2.0,
    };
    let mut is_reverse = false;
    let mut kick_pos: Option<f32> = None;
    let mut snare_pos: Option<f32> = None;
    let mut hat_pos: Option<f32> = None;
    let mut sample_slot_pos: Option<f32> = None;
    let mut bitcrush: i8 = 0;
    let mut tape_factor: f32 = 1.0;

    // 缓冲区, Haas, 常量, 波表
    let audio_buffers = AUDIO_BUFFERS.init([[0u16; HALF_DMA_LEN]; 2]);
    let mut current_buffer_idx = 0;
    let haas_delay_line = HAAS_DELAY_LINE.init([0i16; HAAS_DELAY_SIZE]);
    let mut haas_write_ptr: usize = 0;
    let mut haas_active: bool = false;
    let mut master_drive = 1.;

    const TABLE_MASK: usize = WAVE_TABLE_SIZE - 1;
    const TABLE_SIZE_F32: f32 = WAVE_TABLE_SIZE as f32;
    const TWO_PI: f32 = 2.0 * PI;
    const TWO_PI_INV: f32 = 0.15915494; // 1.0 / (2.0 * PI)
    const I16_SCALE: f32 = 32767.0; // i16 max value for scaling

    let sine_table = get_sine_table();
    let sawtooth_table = get_sawtooth_table();
    let square_table = get_square_table();
    let triangle_table = get_triangle_table();

    let kick_samples = get_kick_sample_table();
    let snare_samples = get_snare_sample_table();
    let hat_samples = get_hat_sample_table();

    let mut master_volume: f32 = 0.8;

    let mut fill_buffer = |buffer: &mut [u16; HALF_DMA_LEN],
                           freq: f32,
                           p: &FmParams,
                           wp: &WaveParams,
                           amp: f32, // 这是 FM 的 amp
                           cp: &mut f32,
                           mp: &mut f32,
                           kick_pos: &mut Option<f32>,
                           snare_pos: &mut Option<f32>,
                           hat_pos: &mut Option<f32>,
                           sample_slot_pos: &mut Option<f32>,
                           on: bool,
                           hdl: &mut [i16; HAAS_DELAY_SIZE],
                           hwp: &mut usize,
                           haas_on: bool,
                           master_drive_val: f32,
                           is_reverse_val: bool,
                           playback_step_val: f32,
                           bitcrush_val: i8,
                           tape_factor_val: f32,
                           global_vol: f32| {

        let effective_freq = freq * tape_factor_val;
        let carrier_freq = freq;
        let modulator_freq = carrier_freq * p.ratio;
        let carrier_phase_increment = (TWO_PI * carrier_freq) / 48000.0;
        let modulator_phase_increment = (TWO_PI * modulator_freq) / 48000.0;
        let carrier_lut = match wp.carrier_wave {
            Waveform::Sine => sine_table,
            Waveform::Triangle => triangle_table,
            Waveform::Sawtooth => sawtooth_table,
            Waveform::Square => square_table,
        };
        let mod_lut = match wp.mod_wave {
            Waveform::Sine => sine_table,
            Waveform::Triangle => triangle_table,
            Waveform::Sawtooth => sawtooth_table,
            Waveform::Square => square_table,
        };

        let fm_total_gain_pre_drive = amp * 0.3;
        let drum_total_gain_pre_drive = 0.3;

        let final_scale_factor = if master_drive_val > 0.0001 {
            5.0 / master_drive_val
        } else {
            5.0
        };

        let mut local_kick_pos = *kick_pos;
        let mut local_snare_pos = *snare_pos;
        let mut local_hat_pos = *hat_pos;
        let mut local_hwp = *hwp;
        let effective_playback_step = playback_step_val * tape_factor_val;
        let effective_drum_step = 1.0 * tape_factor_val;

        for i in 0..SAMPLES_PER_BUFFER {
            let mut drum_sample_f32 = 0.0;
            let fm_sample_f32 = if on {
                let mod_phase_rads = *mp;
                let mod_index_f32 = (mod_phase_rads * TWO_PI_INV) * TABLE_SIZE_F32;
                let mod_idx0 = (mod_index_f32 as i32) as usize & TABLE_MASK;

                let mod_val = mod_lut[mod_idx0];

                let phase_offset = mod_val * p.index;
                let mut carrier_phase_rads = *cp + phase_offset;

                if carrier_phase_rads > TWO_PI {
                    carrier_phase_rads -= TWO_PI;
                }

                let carrier_index_f32 = (carrier_phase_rads * TWO_PI_INV) * TABLE_SIZE_F32;
                let carrier_idx0 = (carrier_index_f32 as i32) as usize & TABLE_MASK;

                carrier_lut[carrier_idx0]
            } else {
                0.0
            };

            // 鼓

            if let Some(pos) = local_kick_pos {
                let idx = pos as usize;
                if idx < KICK_SAMPLE_LEN {
                    drum_sample_f32 += kick_samples[idx];
                    local_kick_pos = Some(pos + effective_drum_step);
                } else {
                    local_kick_pos = None;
                }
            }
            if let Some(pos) = local_snare_pos {
                let idx = pos as usize;
                if idx < SNARE_SAMPLE_LEN {
                    drum_sample_f32 += snare_samples[idx];
                    local_snare_pos = Some(pos + effective_drum_step);
                } else {
                    local_snare_pos = None;
                }
            }
            if let Some(pos) = local_hat_pos {
                let idx = pos as usize;
                if idx < HAT_SAMPLE_LEN {
                    drum_sample_f32 += hat_samples[idx];
                    local_hat_pos = Some(pos + effective_drum_step);
                } else {
                    local_hat_pos = None;
                }
            }
            if let Some(pos) = *sample_slot_pos {
                let sample_len = sample_ready_ref.load(Ordering::SeqCst);

                let read_idx_f32 = if is_reverse_val {
                    (sample_len as f32) - 1.0 - pos
                } else {
                    pos
                };

                let idx = read_idx_f32.max(0.0) as usize;

                if idx < sample_len && pos < (sample_len as f32) {
                    // 读取
                    drum_sample_f32 += sample_buffer[idx] as f32 / 32768.0;
                    // 步进
                    *sample_slot_pos = Some(pos + effective_playback_step);
                } else {
                    *sample_slot_pos = None;
                }
            }

            // 混合
            let pre_driven_signal = (fm_sample_f32 * fm_total_gain_pre_drive)
                + (drum_sample_f32 * drum_total_gain_pre_drive);

            let driven_signal = pre_driven_signal * master_drive_val;

            let saturated_signal = cheap_saturator(driven_signal);

            let final_sample_f32 = saturated_signal * final_scale_factor * global_vol;
 
            let mut mono_sample_i16 = (final_sample_f32 * I16_SCALE) as i16;

            if bitcrush_val != 0 {
                mono_sample_i16 = (mono_sample_i16 >> bitcrush_val) << bitcrush_val;
            }

            // Haas
            let read_ptr = local_hwp;
            let delayed_sample_i16 = hdl[read_ptr];
            hdl[read_ptr] = mono_sample_i16;

            local_hwp += 1;
            if local_hwp >= HAAS_DELAY_SIZE {
                local_hwp = 0;
            }
            let left = mono_sample_i16 as u16;
            let right = if haas_on {
                delayed_sample_i16 as u16
            } else {
                left
            };

            buffer[i * 2] = left;
            buffer[i * 2 + 1] = right;

            // 相位推进
            *cp += carrier_phase_increment;
            *mp += modulator_phase_increment;

            if *cp > TWO_PI {
                *cp -= TWO_PI;
            }
            if *mp > TWO_PI {
                *mp -= TWO_PI;
            }
        }

        // 回写状态
        *kick_pos = local_kick_pos;
        *snare_pos = local_snare_pos;
        *hat_pos = local_hat_pos;
        *hwp = local_hwp;

        if !on {
            *cp = 0.0;
            *mp = 0.0;
        }
    }; // (fill_buffer 闭包结束)
    fill_buffer(
        &mut audio_buffers[0],
        frequency,
        &params,
        &wave_params,
        amplitude,
        &mut carrier_phase,
        &mut modulator_phase,
        &mut kick_pos,
        &mut snare_pos,
        &mut hat_pos,
        &mut sample_slot_pos,
        is_on,
        haas_delay_line,
        &mut haas_write_ptr,
        haas_active,
        master_drive,
        is_reverse,
        playback_step,
        bitcrush,
        tape_factor,
        master_volume,
    );
    fill_buffer(
        &mut audio_buffers[1],
        frequency,
        &params,
        &wave_params,
        amplitude,
        &mut carrier_phase,
        &mut modulator_phase,
        &mut kick_pos,
        &mut snare_pos,
        &mut hat_pos,
        &mut sample_slot_pos,
        is_on,
        haas_delay_line,
        &mut haas_write_ptr,
        haas_active,
        master_drive,
        is_reverse,
        playback_step,
        bitcrush,
        tape_factor,
        master_volume,
    );

    i2s.start();
    info!("I2S started!");
    let mut write_future = i2s.write(&audio_buffers[current_buffer_idx]);

    loop {
        if let Ok(rev) = REVERSE_STATE_CHANNEL.try_receive() {
            is_reverse = rev;
        }
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
                kick_pos = None;
                snare_pos = None;
                hat_pos = None;
                write_future = i2s.write(&audio_buffers[current_buffer_idx]);
                continue;
            }
        }

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

        // 消息接受填充
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
        if let Ok(speed) = SAMPLE_PITCH_CHANNEL.try_receive() {
            playback_step = speed;
        }
        if let Ok(bit) = BITCRUSH_CHANNEL.try_receive() {
            bitcrush = bit;
        }
        if let Ok(tf) = TAPE_FACTOR_CHANNEL.try_receive() {
            tape_factor = tf;
        }
        if let Ok(vol) = MASTER_VOLUME_CHANNEL.try_receive() {
            master_volume = vol;
        }

        while let Ok(drum) = synth::DRUM_CHANNEL.try_receive() {
            match drum {
                DrumSample::Kick => kick_pos = Some(0.),
                DrumSample::Snare => snare_pos = Some(0.),
                DrumSample::Hat => hat_pos = Some(0.),
                DrumSample::User => sample_slot_pos = Some(0.),
            }
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

        fill_buffer(
            buf_to_fill,
            frequency,
            &params,
            &wave_params,
            amplitude,
            &mut carrier_phase,
            &mut modulator_phase,
            &mut kick_pos,
            &mut snare_pos,
            &mut hat_pos,
            &mut sample_slot_pos,
            is_on,
            haas_delay_line,
            &mut haas_write_ptr,
            haas_active,
            master_drive,
            is_reverse,
            playback_step,
            bitcrush,
            tape_factor,
            master_volume,
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
