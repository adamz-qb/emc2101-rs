pub type Result<T, E> = core::result::Result<T, Error<E>>;

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C bus error.
    I2c(E),
    /// The device Product ID is not supported.
    InvalidID,
    /// The given Value is not valid.
    InvalidValue,
    /// The given array as an invalid size.
    InvalidSize,
    /// The given array is not sorted.
    InvalidSorting,
    /// Errors such as overflowing the stack.
    Internal,
}

// impl core::error::Error for Error {}

// impl core::fmt::Display for Error {
//     fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
//         write!(f, "{self:?}")
//     }
// }
