# SCServo Linux SDK subset

The SCS0009, SCS and SCSerial sources and matching public headers come from
[SCServo_Linux](https://github.com/adityakamath/SCServo_Linux/tree/25e86a8dd67728581716e14c15c7384f29cb6ee6).
The shared helpers include the protocol types directly so other motor families
are unnecessary. Cached SCS0009 feedback words use the same byte order as
direct register reads. See LICENSE for the original MIT terms.
