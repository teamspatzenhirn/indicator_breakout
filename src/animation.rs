use core::cmp::min;
use smart_leds::{colors, RGB8};

use fugit::ExtU64;

use crate::IndicatorState;

pub type Duration = <rp2040_hal::timer::monotonic::Monotonic<rp2040_hal::timer::Alarm0> as rtic::Monotonic>::Duration;

pub const STRIP_LEN: usize = 16 * 3;

pub enum AnimationResult {
    NextFrameIn(Duration),
    // Returned for last frame
    Done(Duration),
}

pub fn calculate_frame(
    buf: &mut [RGB8; STRIP_LEN],
    i: u32,
    mode: &IndicatorState,
    brake: bool,
) -> AnimationResult {
    buf.fill(colors::BLACK);

    let animation_duration: Duration = 500.millis();
    let pixel_duration: Duration = 21.millis();
    let hold_duration = animation_duration - 16 * pixel_duration;

    let result = match mode {
        IndicatorState::LEFT => match i {
            0..=15 => {
                let progress = min(16, i) as usize;
                buf[32..32 + progress].fill(colors::YELLOW);
                AnimationResult::NextFrameIn(pixel_duration)
            }
            16 => {
                let progress = min(16, i) as usize;
                buf[32..32 + progress].fill(colors::YELLOW);
                AnimationResult::NextFrameIn(hold_duration)
            }
            _ => AnimationResult::Done(0.millis()),
        },
        IndicatorState::RIGHT => match i {
            0..=15 => {
                let progress = min(16, i) as usize;
                buf[16 - progress..16].fill(colors::YELLOW);
                AnimationResult::NextFrameIn(pixel_duration)
            }
            16 => {
                let progress = min(16, i) as usize;
                buf[16 - progress..16].fill(colors::YELLOW);
                AnimationResult::NextFrameIn(hold_duration)
            }

            _ => AnimationResult::Done(0.millis()),
        },
        IndicatorState::BOTH => match i {
            0 => {
                buf[0..16].fill(colors::YELLOW);
                buf[STRIP_LEN - 16..STRIP_LEN].fill(colors::YELLOW);
                AnimationResult::NextFrameIn(animation_duration / 2)
            }
            _ => AnimationResult::Done(animation_duration / 2),
        },
        IndicatorState::OFF => AnimationResult::Done(0.millis()),
    };

    if brake {
        buf[0..3].fill(colors::RED);
        buf[STRIP_LEN - 3..STRIP_LEN].fill(colors::RED);
        buf[16..32].fill(colors::RED);
    };

    result
}
