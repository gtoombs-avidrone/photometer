Avidrone Software Take-Home
===========================

Introduction
------------

This is a question specification for a take-home project. Take-homes are valuable hiring tools
because they give candidates the time to shine in a somewhat-realistic software development
environment. Avidrone uses this as a first-stage supplement (not replacement) to a technical
interview.

We published the take-home reference implementation and tests to this public repository to allow
former candidates to check their work, and for other people to have fun with it. This take-home
will no longer be used to evaluate developer candidacy, but future take-homes will follow a
somewhat similar theme and format.

Submission Format
-----------------

Assume that only the
[C](https://en.wikipedia.org/wiki/C_standard_library) and
[C++](https://en.wikipedia.org/wiki/Standard_Template_Library) standard libraries are available.
Choose and specify your C++ standard version. You cannot assume the presence of a particular
operating system or compiler.

Evaluation Criteria
-------------------

Responses to the question are evaluated on:

- correctness and maintainability of the solution
- performance and scalability
- adherence to industrial standards

Plagiarised or AI-generated content are not acceptable.

Question
--------

You are writing firmware for a solar-powered blimp. As part of the sensor suite for the blimp, the
aircraft motherboard includes a very strange relative photometer. You need to write a driver that
runs on the host, decodes frames from the sensor, and produces illuminance estimates.

Sensor
------

Here is the specification for the photometer:

> **DIMBULB2001 PHOTOMETER**
> 
> The DIMBULB uses an innovative Multiphasic Array Incremental Sensor Estimator (MAISE).
> 
> Instead of producing direct measurements of illuminance, the DIMBULB produces lower and upper
> bound estimates based on varying light conditions. These bound updates allow the connected host
> to converge on a measure of central tendency. Frames are sent from the photometer to the host at
> a variable rate of 500 Hz to 10 kHz depending on bus configuration.
> 
> Each frame is sent in little-endian format as
> 
> | 15   | 14   | 13   | 12   | 11   | 10   | 9    | 8    | 7    | 6    | 5    | 4    | 3    | 2   | 1    | 0    |
> |------|------|------|------|------|------|------|------|------|------|------|------|------|-----|------|------|
> | HRZ3 | HRZ2 | HRZ1 | HRZ0 | SIGN | VAL7 | VAL6 | VAL5 | VAL4 | VAL3 | VAL2 | VAL1 | VAL0 | CLR | CNF1 | CNF0 |  
>
> HRZ(3:0) is a time horizon of the estimate, where the time is expressed with the exponential
> 
>     t_valid = 16.5*2^HRZ  (ms)
> 
> From the time of this sample (inclusive) through the time horizon (exclusive), the host should
> take this sample into account. When the time horizon expires, the host should discard this
> sample.
> 
> SIGN is 1 if this sample's value is greater than the estimated true value, or 0 if this
> sample's value is lower than the estimated true value.
> 
> VAL(7:0) is a signed two's-complement value representing the offset in units of 390 lx from the
> reference illuminance. The reference illuminance is factory-calibrated to 50,000 lx.
> 
> If CLR is 1, the host should discard the effects of all prior samples (the content of this sample
> is still valid).
> 
> CNF(1:0) is a confidence metric from 0-3. If the photometer issues two conflicting samples, the
> sample with the higher confidence should override the sample with lower confidence. If two
> samples having the same confidence conflict, the newer sample should override the older sample.

Software Interface
------------------

Write a class with a signature having at minimum the following methods:

```cpp
class Photometer
{
public:
    // This signature cannot change
    void consume(
        double now,            // monotonic seconds
        const uint8_t data[2]  // raw from the sensor
    );
    
    // This signature cannot change
    double estimate(
        double now  // monotonic seconds
    ) const;
};
```

The caller is responsible for providing you a valid timestamp in fractional seconds. You may assume
that the timestamp is monotonic over multiple calls.
