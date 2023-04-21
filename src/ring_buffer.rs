use core::ops::{Deref, DerefMut};

pub(crate) struct RingBuffer<T, const N: usize> {
    inner: [T; N],
    pos: usize,
}

impl<T, const N: usize> Deref for RingBuffer<T, N> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<T, const N: usize> DerefMut for RingBuffer<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<T, const N: usize> RingBuffer<T, N> {
    pub(crate) fn new(init: [T; N]) -> Self {
        Self {
            pos: init.len(),
            inner: init,
        }
    }

    pub(crate) fn push(&mut self, x: T) -> T {
        self.pos = (self.pos + 1) % self.len();
        core::mem::replace(&mut self.inner[self.pos], x)
    }
}
