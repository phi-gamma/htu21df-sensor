use crate::Error;
use std::{error, fmt};

impl<E: fmt::Debug> error::Error for Error<E> {}

impl<E: fmt::Debug> fmt::Display for Error<E>
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result
    {
        write!(f, "{:?}", self)
    }
}
