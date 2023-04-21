use core::cell::UnsafeCell;
use core::panic::PanicInfo;
use core::ptr;
use core::sync::atomic::{self, Ordering};

use cortex_m::delay::Delay;
use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::gpio::bank0::Gpio25;
use rp2040_hal::gpio::{Output, Pin, PushPull};
use rp2040_hal::sio::Spinlock0;

#[repr(transparent)]
pub struct SyncUnsafeCell<T: ?Sized> {
    value: UnsafeCell<T>,
}

unsafe impl<T: ?Sized + Sync> Sync for SyncUnsafeCell<T> {}

impl<T> SyncUnsafeCell<T> {
    /// Constructs a new instance of `SyncUnsafeCell` which will wrap the specified value.
    #[inline]
    pub const fn new(value: T) -> Self {
        Self {
            value: UnsafeCell::new(value),
        }
    }
}

impl<T> SyncUnsafeCell<T> {
    /// Gets a mutable pointer to the wrapped value.
    ///
    /// This can be cast to a pointer of any kind.
    /// Ensure that the access is unique (no active references, mutable or not)
    /// when casting to `&mut T`, and ensure that there are no mutations
    /// or mutable aliases going on when casting to `&T`
    #[inline]
    pub const fn get(&self) -> *mut T {
        self.value.get()
    }
}

type PanicPin = Option<Pin<Gpio25, Output<PushPull>>>;

const NO_PIN: PanicPin = None;

static PANIC_PIN: SyncUnsafeCell<PanicPin> = SyncUnsafeCell::new(NO_PIN);

pub(crate) fn set_panic_pin(mut pin: Pin<Gpio25, Output<PushPull>>, delay: &mut Delay) {
    if let Some(_lock) = Spinlock0::try_claim() {
        let _ = pin.set_high();
        delay.delay_ms(50);
        let _ = pin.set_low();
        delay.delay_ms(50);
        let _ = pin.set_high();
        delay.delay_ms(50);
        let _ = pin.set_low();
        unsafe { ptr::write(PANIC_PIN.get(), Some(pin)) };
    }
}

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let lock = Spinlock0::try_claim();
    if lock.is_some() {
        if let Some(mut pin) = unsafe { ptr::replace(PANIC_PIN.get(), NO_PIN) } {
            let _ = pin.set_high();
        }
    }

    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
