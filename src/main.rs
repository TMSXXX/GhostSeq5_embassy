#![no_std]
#![no_main]

use core::fmt::Write;
use defmt::*;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_futures::select::{Either, select};
use once_cell::sync::OnceCell;
use embassy_stm32::{
    gpio::{AnyPin, Input, Level, Output, Pull, Speed},
    i2c::I2c,
    i2s,
    interrupt::{self, InterruptExt, Priority}, // 确保导入 Priority
    time::Hertz,
    Peri,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex}, // 导入 CriticalSectionRawMutex
    channel::Channel,
};
use embassy_time::Timer;
use heapless::String;
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
use micromath::F32Ext;
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

type OledDisplay = Ssd1306<
    I2CInterface<I2c<'static, embassy_stm32::mode::Blocking, embassy_stm32::i2c::Master>>,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
>;
type WaveformBuffer = ([i32; 128], f32);
type OledText = String<16>;

// 使用官方示例的缓冲区大小
const AUDIO_DMA_BUF_SIZE: usize = 2400;
static DMA_BUF_CELL: StaticCell<[u16; AUDIO_DMA_BUF_SIZE]> = StaticCell::new();
const HALF_DMA_LEN: usize = AUDIO_DMA_BUF_SIZE / 2; // 1200
const SAMPLES_PER_BUFFER: usize = HALF_DMA_LEN / 2; // 600

const SINE_TABLE_SIZE: usize = 1024;

const NOTE_FREQUENCIES: [f32; 8] = [
    // --- C4 八度 ---
    261.63, // C4 (Do) - 按键 0
    293.66, // D4 (Re) - 按键 1
    329.63, // E4 (Mi) - 按键 2
    349.23, // F4 (Fa) - 按键 3
    392.00, // G4 (So) - 按键 4
    440.00, // A4 (La) - 按键 5
    493.88, // B4 (Ti) - 按键 6
    523.25, // C5 (Do) - 按键 7
];

// 这是一个可以被安全地初始化一次的 static 单元
static SINE_TABLE: OnceCell<[f32; SINE_TABLE_SIZE]> = OnceCell::new();

// 我们创建一个辅助函数来获取（或在首次调用时创建）表
fn get_sine_table() -> &'static [f32; SINE_TABLE_SIZE] {
    // get_or_init 会在第一次被调用时运行这个闭包来创建表
    // 并且保证这个闭包只会被运行一次。
    // 在那之后，它只会立即返回已创建的表的引用。
    SINE_TABLE.get_or_init(|| {
        let mut table = [0.0f32; SINE_TABLE_SIZE];
        for i in 0..SINE_TABLE_SIZE {
            let phase = (i as f32 / SINE_TABLE_SIZE as f32) * (2.0 * PI);
            table[i] = phase.sin(); 
        }
        info!("Sine table (1024 samples) generated.");
        table
    })
}

// 定义消息类型
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

// --------------------------------------------------------------------------
// 修复 1: 将所有互斥锁更改为 CriticalSectionRawMutex
// 这允许中断（音频）和线程模式（其他）安全通信
// --------------------------------------------------------------------------
static KEYBOARD_CHANNEL: Channel<CriticalSectionRawMutex, Keyboard, 8> = Channel::new();
static FREQUENCY_CHANNEL: Channel<CriticalSectionRawMutex, f32, 2> = Channel::new(); // 只发送 f32
static OLED_TEXT_CHANNEL: Channel<CriticalSectionRawMutex, OledText, 2> = Channel::new();
static AUDIO_CHANNEL: Channel<CriticalSectionRawMutex, AudioCommand, 4> = Channel::new();

// --------------------------------------------------------------------------
// 修复 1: 按照官方示例，定义裸露的 InterruptExecutor
// 不需要 StaticCell，也不需要泛型
// --------------------------------------------------------------------------
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();

// --------------------------------------------------------------------------
// 修复 2: 绑定中断函数
// --------------------------------------------------------------------------
#[embassy_stm32::interrupt]
unsafe fn TIM2() {
    EXECUTOR_HIGH.on_interrupt();
}

#[embassy_stm32::interrupt] // <-- 新增
unsafe fn TIM3() {
    EXECUTOR_MED.on_interrupt();
}
#[embassy_executor::main]
async fn main(spawner: Spawner) { // 这个 spawner 是 LOW 优先级的
    enable_fpu();
    let config = {
        // ... (config 结构体完全不变) ...
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

    
    // 初始化I2C和OLED
   let mut i2c_config = embassy_stm32::i2c::Config::default();
    i2c_config.frequency = Hertz(400_000); // 建议 400k 以减少阻塞

    let i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c_config);
    info!("I2C initialized.");

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    info!("OLED display created.");

    let keys: [[Peri<'static, AnyPin>; 4]; 2] = [
        [p.PA0.into(), p.PA1.into(), p.PA2.into(), p.PA3.into()],
        [p.PA4.into(), p.PA5.into(), p.PA6.into(), p.PA7.into()],
    ];
    info!("Keyboard pins configured.");

    // 使用官方示例的I2S配置
    let mut i2s_config = i2s::Config::default();
    i2s_config.format = i2s::Format::Data16Channel32;
    i2s_config.master_clock = false;
    i2s_config.frequency = Hertz(48000); // 改为48000Hz

    info!("I2S config set.");

    // I2S初始化 
    let i2s = i2s::I2S::new_txonly_nomck(
        p.SPI3,     // SPI3
        p.PB5,      // sd
        p.PA15,     // ws
        p.PB3,      // ck
        p.DMA1_CH7, // txdma
        DMA_BUF_CELL.init([0u16; AUDIO_DMA_BUF_SIZE]),
        i2s_config,
    );
    info!("I2S configured.");

    // 初始化OLED
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap();
    display.flush().unwrap();

    // --------------------------------------------------------------------------
    // 修复 3: 设置和启动 HIGH (P3)
    // --------------------------------------------------------------------------
    interrupt::TIM2.set_priority(Priority::P3);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::TIM2);
    
    // --------------------------------------------------------------------------
    // 修复 4: 设置和启动 MEDIUM (P7)
    // --------------------------------------------------------------------------
    interrupt::TIM3.set_priority(Priority::P7); // P7 高于 P15, 低于 P3
    let spawner_med = EXECUTOR_MED.start(interrupt::TIM3);

    // --------------------------------------------------------------------------
    // 修复 5: 重新分配任务
    // --------------------------------------------------------------------------
    
    // LOW (P15): 只有 OLED 任务，它现在可以随意阻塞
    spawner.spawn(oled_task(display)).unwrap();

    // MEDIUM (P7): 实时输入
    spawner_med.spawn(keyboard_task(keys)).unwrap();
    spawner_med.spawn(synth_task()).unwrap();

    // HIGH (P3): 实时音频
    spawner_high.spawn(audio_task(i2s)).unwrap();

    info!("All tasks started, system ready!");
}

fn enable_fpu() {
    unsafe {
        let scb = cortex_m::peripheral::SCB::ptr();
        (*scb).cpacr.modify(|r| r | (0b1111 << 20));
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

// keyboard_task 保持不变 (使用 CriticalSectionRawMutex)
#[embassy_executor::task]
async fn keyboard_task(keys: [[Peri<'static, AnyPin>; 4]; 2]) {
    info!("Keyboard task started!");
    let [rows, cols] = keys.map(|line| line);
    let mut rows = rows.map(|pin| Output::new(pin, Level::High, Speed::Low));
    let cols = cols.map(|pin| Input::new(pin, Pull::Up));
    let mut last_key_state: [bool; 16] = [false; 16];

    loop {
        let mut current_key_state: [bool; 16] = [false; 16];

        for (r, row) in rows.iter_mut().enumerate() {
            row.set_low();
            Timer::after_micros(10).await; // 确保 await 足够短

            for (c, col) in cols.iter().enumerate() {
                if col.is_low() {
                    let key_code = (r * 4 + c) as usize;
                    current_key_state[key_code] = true;
                }
            }
            row.set_high();
        }

        for i in 0..16 {
            let key_code = i as u8;
            if current_key_state[i] && !last_key_state[i] {
                info!("Key Pressed: {}", key_code);
                let _ = KEYBOARD_CHANNEL.try_send(Keyboard::KeyPress(key_code));
            } else if !current_key_state[i] && last_key_state[i] {
                info!("Key Released: {}", key_code);
                let _ = KEYBOARD_CHANNEL.try_send(Keyboard::KeyRelease(key_code));
            }
        }

        last_key_state = current_key_state;
        Timer::after_millis(5).await; // 20ms 轮询间隔
    }
}

// oled_task 保持不变 (使用 CriticalSectionRawMutex)
#[embassy_executor::task]
async fn oled_task(mut display: OledDisplay) {
    info!("OLED task started!");

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let clear_style = PrimitiveStyle::with_fill(BinaryColor::Off);

    // 状态变量
    let mut frequency: f32 = 0.0;
    let mut key_text: String<16> = String::new();
    core::write!(key_text, "Init...").unwrap();

    // 标志位
    let mut freq_dirty = true;
    let mut text_dirty = true;

    // 初始显示
    display.clear(BinaryColor::Off).unwrap();
    // 第一次 flush 必须在循环外，以显示 "Init..."
    // (修复：确保初始文本也被绘制)
    Rectangle::new(Point::new(0, 34), Size::new(128, 12)) // (y=34, h=12)
        .into_styled(clear_style)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline(&key_text, Point::new(0, 34), text_style, Baseline::Top) // (y=34)
        .draw(&mut display)
        .unwrap();
    display.flush().unwrap();
    text_dirty = false; // 我们已经画过了


    loop {
        // --- 1. 非阻塞地排空所有挂起的消息 ---
        if let Ok(new_frequency) = FREQUENCY_CHANNEL.try_receive() {
            frequency = new_frequency;
            freq_dirty = true;
        }
        if let Ok(new_text) = OLED_TEXT_CHANNEL.try_receive() {
            key_text = new_text;
            text_dirty = true;
        }

        // --- 2. 仅在需要时重绘 ---
        
        if freq_dirty {
            let mut freq_text: String<16> = String::new();
            core::write!(freq_text, "Freq: {} Hz", frequency as i32).unwrap();
            
            // -----------------------------------------------------------------
            // 修复: 清除矩形的 Y 坐标必须匹配文本
            // 我们清除 y=10 处，高度 12 (10px 字体 + 2px 留白)
            // -----------------------------------------------------------------
            Rectangle::new(Point::new(0, 10), Size::new(128, 12)) 
                .into_styled(clear_style)
                .draw(&mut display)
                .unwrap();
            Text::with_baseline(&freq_text, Point::new(0, 10), text_style, Baseline::Top) 
                .draw(&mut display)
                .unwrap();
        }

        if text_dirty {
             // -----------------------------------------------------------------
             // 修复: 清除矩形的 Y 坐标必须匹配文本
             // 我们清除 y=34 处，高度 12 (10px 字体 + 2px 留白)
             // -----------------------------------------------------------------
             Rectangle::new(Point::new(0, 34), Size::new(128, 12)) 
                .into_styled(clear_style)
                .draw(&mut display)
                .unwrap();
            Text::with_baseline(&key_text, Point::new(0, 34), text_style, Baseline::Top) 
                .draw(&mut display)
                .unwrap();
        }

        // --- 3. 仅在有新内容时才阻塞刷新 ---
        if freq_dirty || text_dirty {
            display.flush().unwrap(); 
        }
        
        freq_dirty = false;
        text_dirty = false;

        // --- 4. Await (释放 CPU) ---
        Timer::after_millis(10).await;
    }
}

// synth_task 保持不变 (使用 CriticalSectionRawMutex)
#[embassy_executor::task]
async fn synth_task() {
    info!("Synth task started!");

    let mut current_frequency = 0.0f32;
    let mut last_frequency = 0.0f32;

    // --------------------------------------------------------------------------
    // 修复 2: 添加“切换”状态
    // --------------------------------------------------------------------------
    let mut is_sharp_active = false; // 我们的“切换”状态
    const SHARP_KEY_ID: u8 = 8; // 按键 8 是我们的“切换”键
    const SEMITONE_UP: f32 = 1.0594635; // 2^(1/12)

    // 发送初始状态
    let _ = FREQUENCY_CHANNEL.try_send(0.0); // 启动时频率为 0
    let mut status_text: String<16> = String::new();
    core::write!(status_text, "Ready").unwrap();
    let _ = OLED_TEXT_CHANNEL.try_send(status_text);

    loop {
        match KEYBOARD_CHANNEL.receive().await {
            Keyboard::KeyPress(key) => {
                
                if key == SHARP_KEY_ID {
                    // --- 按下了“切换”键 (Key 8) ---
                    is_sharp_active = !is_sharp_active; // 翻转状态
                    info!("Sharp Toggled: {}", is_sharp_active);
                    
                    let mut text_to_send: String<16> = String::new();
                    if is_sharp_active {
                        core::write!(text_to_send, "Sharp: ON").unwrap();
                    } else {
                        core::write!(text_to_send, "Sharp: OFF").unwrap();
                    }
                    let _ = OLED_TEXT_CHANNEL.try_send(text_to_send);
                
                } else if key <= 7 {
                    // --- 这是一个“音符键” (0-7) ---
                    info!("Note Key {} pressed", key);
                    
                    let base_frequency = NOTE_FREQUENCIES[key as usize];
                    
                    current_frequency = if is_sharp_active {
                        base_frequency * SEMITONE_UP
                    } else {
                        base_frequency
                    };

                    if (current_frequency - last_frequency).abs() > 1.0 {
                        let mut text_to_send: String<16> = String::new();
                        core::write!(text_to_send, "Note {} On", key).unwrap();
                        let _ = OLED_TEXT_CHANNEL.try_send(text_to_send);
                        
                        // 发送音频命令
                        let _ = AUDIO_CHANNEL.try_send(AudioCommand::Play(current_frequency));
                        
                        // ----------------------------------------------------------
                        // 修复 3: 移除波形计算，只发送频率
                        // ----------------------------------------------------------
                        let _ = FREQUENCY_CHANNEL.try_send(current_frequency);
                        last_frequency = current_frequency;
                    }
                    
                    // (波形图计算已 100% 移除，P7 负担大大减轻)
                
                } else {
                    // --- 这是一个“其他功能键” (9-15) ---
                    info!("Function Key {} pressed", key);
                    let mut text_to_send: String<16> = String::new();
                    core::write!(text_to_send, "Func Key {}", key).unwrap();
                    let _ = OLED_TEXT_CHANNEL.try_send(text_to_send);
                }
            }
            Keyboard::KeyRelease(key) => {
                
                if key == SHARP_KEY_ID {
                    // --- 松开了“切换”键 (Key 8) ---
                    info!("Sharp key released (no action)");
                    // “切换”逻辑下，松开按键不做任何事

                } else if key <= 7 {
                    // --- 松开了“音符键” (0-7) ---
                    info!("Note Key {} released", key);

                    let mut text_to_send: String<16> = String::new();
                    core::write!(text_to_send, "Note {} Off", key).unwrap();
                    let _ = OLED_TEXT_CHANNEL.try_send(text_to_send);
                    
                    let _ = AUDIO_CHANNEL.try_send(AudioCommand::Stop);
                    last_frequency = 0.0;
                    current_frequency = 0.0;
                    
                    // ----------------------------------------------------------
                    // 修复 3 (续): 发送 0.0 频率到 OLED
                    // ----------------------------------------------------------
                    let _ = FREQUENCY_CHANNEL.try_send(0.0);
                
                } else {
                    // --- 松开了“其他功能键” (9-15) ---
                    info!("Function Key {} released", key);
                }
            }
        }
        Timer::after_millis(1).await;
    }
}

// --------------------------------------------------------------------------
// 修复 3: 更新 audio_task 以使用正确的缓冲区大小 (HALF_DMA_LEN)
// --------------------------------------------------------------------------
#[embassy_executor::task()]
async fn audio_task(mut i2s: i2s::I2S<'static, u16>) {
    info!("Audio task starting...");

    let mut frequency = 100.0f32;
    let mut phase: f32 = 0.0; // 0.0 到 2.0*PI 弧度
    let mut is_on = false;

    let mut audio_buffers = [[0u16; HALF_DMA_LEN], [0u16; HALF_DMA_LEN]];
    let mut current_buffer_idx = 0;

    // (fill_buffer 和 get_sine_table 保持不变)
    let mut fill_buffer =
        |buffer: &mut [u16; HALF_DMA_LEN], freq: f32, p: &mut f32, on: bool| {
            
            let sine_table = get_sine_table();
            let phase_increment = (2.0 * PI * freq) / 48000.0;
            
            for i in 0..SAMPLES_PER_BUFFER {
                let sample_f32 = if on {
                    const TABLE_SIZE_F32: f32 = SINE_TABLE_SIZE as f32;
                    let index_f32 = (*p / (2.0 * PI)) * TABLE_SIZE_F32;
                    let index_trunc = index_f32.floor(); 
                    let index_frac = index_f32 - index_trunc; 
                    let idx0 = index_trunc as usize;
                    let idx1 = (idx0 + 1) % SINE_TABLE_SIZE; 
                    let val0 = sine_table[idx0];
                    let val1 = sine_table[idx1];
                    (val1 - val0).mul_add(index_frac, val0) 
                } else {
                    0.0
                };
                let sample = (sample_f32 * 0.8 * 32767.0) as i16;
                buffer[i * 2] = sample as u16; 
                buffer[i * 2 + 1] = sample as u16;
                *p += phase_increment;
                if *p > (2.0 * PI) {
                    *p -= (2.0 * PI);
                }
            }
            if !on { *p = 0.0; }
        };

    // 预填充两个缓冲区
    fill_buffer(&mut audio_buffers[0], frequency, &mut phase, is_on);
    fill_buffer(&mut audio_buffers[1], frequency, &mut phase, is_on);

    i2s.start();
    info!("I2S started!");
    
    let mut write_future = i2s.write(&audio_buffers[current_buffer_idx]);

    loop {
        // 1. Await 等待上一个传输 (B0) 完成
        match write_future.await {
            Ok(_) => {}
            Err(e) => {
                error!("I2S write error: {:?}", e); 
                // ... (错误处理部分不变) ...
                audio_buffers[0].fill(0); audio_buffers[1].fill(0);
                phase = 0.0; current_buffer_idx = 0; i2s.clear();
                fill_buffer(&mut audio_buffers[0], frequency, &mut phase, is_on);
                fill_buffer(&mut audio_buffers[1], frequency, &mut phase, is_on);
                write_future = i2s.write(&audio_buffers[current_buffer_idx]);
                continue; 
            }
        }
        
        // 2. 拆分缓冲区
        let (buf0_slice, buf1_slice) = audio_buffers.split_at_mut(1);
        let buf0 = &mut buf0_slice[0];
        let buf1 = &mut buf1_slice[0];

        // 3. 切换索引
        current_buffer_idx = (current_buffer_idx + 1) % 2;
        
        let (buf_to_write, buf_to_fill);
        if current_buffer_idx == 0 {
            buf_to_write = buf0; // B0 (即将播放)
            buf_to_fill = buf1;  // B1 (刚刚播完)
        } else {
            buf_to_write = buf1; // B1 (即将播放)
            buf_to_fill = buf0;  // B0 (刚刚播完)
        }

        // 4. 【关键修复】在播放 *之前* 检查新命令
        while let Ok(command) = AUDIO_CHANNEL.try_receive() {
            match command {
                AudioCommand::Play(freq) => {
                    info!("Audio: Play {} Hz", freq);
                    frequency = freq;
                    is_on = true;
                }
                AudioCommand::Stop => {
                    info!("Audio: Stop");
                    is_on = false;
                }
            }
        }

        // 5. 【关键修复】如果刚收到 Stop，
        //    立即将“即将播放”的缓冲区 (buf_to_write) 清零。
        if !is_on {
            buf_to_write.fill(0); // "扼杀"即将播放的缓冲区
        }
        
        // 6. 立即开始下一个传输 (现在可以保证是静音了)
        write_future = i2s.write(buf_to_write);

        // 7. 在 B1 播放时，填充 B0
        fill_buffer(buf_to_fill, frequency, &mut phase, is_on);
    }
}