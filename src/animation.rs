use core::cmp::min;
use smart_leds::{colors, RGB8};

use indicator_interface::IndicatorState;

pub type Duration = <rp2040_hal::timer::monotonic::Monotonic<rp2040_hal::timer::Alarm0> as rtic::Monotonic>::Duration;
type Instant = <rp2040_hal::timer::monotonic::Monotonic<rp2040_hal::timer::Alarm0> as rtic::Monotonic>::Instant;

use fugit::ExtU64;

pub const ANIMATION_DURATION_ms: u64 = 500;
pub const FRAME_DURATION_ms: u64 = 15;
pub const STRIP_LEN: usize = 16 * 3;

pub fn calculate_frame(buf: &mut [RGB8; STRIP_LEN], t: u32, mode: &IndicatorState, brake: bool) {
    buf.fill(colors::BLACK);

    let ANIMATION_DURATION:Duration = Duration::millis(ANIMATION_DURATION_ms);
    let FRAME_DURATION:Duration = Duration::millis(FRAME_DURATION_ms);

    match mode {
        IndicatorState::LEFT => {
            let progress = min(16, t) as usize;
            buf[32..32 + progress].fill(colors::YELLOW);
        }
        IndicatorState::RIGHT => {
            let progress = min(16, t) as usize;
            buf[16 - progress..16].fill(colors::YELLOW);
        }
        IndicatorState::BOTH => {
            buf[0..16].fill(
                if (t as u64) < ((ANIMATION_DURATION / FRAME_DURATION) / 2) {
                    colors::YELLOW
                } else {
                    colors::BLACK
                },
            );

            buf[STRIP_LEN - 16..STRIP_LEN].fill(
                if (t as u64) < (ANIMATION_DURATION / FRAME_DURATION) / 2 {
                    colors::YELLOW
                } else {
                    colors::BLACK
                },
            );
        }
        IndicatorState::OFF => {}
    }

    if brake {
        buf[0..3].fill(colors::RED);
        buf[STRIP_LEN - 3..STRIP_LEN].fill(colors::RED);
        buf[16..32].fill(colors::RED);
    }
}
