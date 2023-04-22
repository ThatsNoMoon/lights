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
			pos: 0,
			inner: init,
		}
	}

	pub(crate) fn push(&mut self, x: T) -> T {
		let old = core::mem::replace(&mut self.inner[self.pos], x);
		self.pos = (self.pos + 1) % self.len();
		old
	}

	pub(crate) fn in_order_iter(&self) -> impl Iterator<Item = &'_ T> + DoubleEndedIterator {
		self.inner[..self.pos]
			.iter()
			.rev()
			.chain(self.inner[self.pos..].iter().rev())
	}
}
