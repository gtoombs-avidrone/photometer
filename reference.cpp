#if __cplusplus < 202002L
#   warning This program was written for compatibility with c++20 or later.
#endif

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>


namespace {

struct RawSample {
    uint16_t confidence: 2;
    uint16_t clear: 1;
    int16_t value: 8;
    uint16_t sign: 1;
    uint16_t horizon: 4;

    void dump() const {
        const uint8_t *bytes = reinterpret_cast<const uint8_t*>(this);
        std::cout
            << "meter.consume(now, (const uint8_t[2]){ "
            << std::hex
            << "0x" << static_cast<uint16_t>(bytes[0]) << "u, "
            << "0x" << static_cast<uint16_t>(bytes[1]) << "u });  //"
            << std::dec
            << " conf=" << confidence
            << " clear=" << clear
            << " value=" << value_lx()
            << " sign=" << sign
            << " horizon=" << horizon_s()
            << "\n";
    }

    constexpr double value_lx() const {
        return 50e3 + 390*value;
    }

    constexpr double horizon_s() const {
        return 0.0165*(1u << horizon);
    }
};

class Sample {
private:
    static constexpr size_t kBytes = 2;

public:
    static constexpr Sample from_raw(
        double now,                    // monotonic seconds
        const uint8_t (&data)[kBytes]  // raw from the sensor
    ) {
        RawSample raw;
        static_assert(sizeof(raw) >= kBytes);
        static_assert(sizeof(raw) <= kBytes);
        std::memcpy(&raw, data, kBytes);
        return Sample(now, raw);
    }

    constexpr Sample(
        double now,           // monotonic seconds
        const RawSample &raw  // raw from the sensor
    ):
        start_(now),
        end_(now + raw.horizon_s()),
        sign_(raw.sign),
        value_(raw.value_lx()),
        clear_(raw.clear),
        confidence_(raw.confidence)
    { }

    constexpr RawSample raw() const {
        return RawSample {
            .confidence = confidence_,
            .clear = clear_,
            .value = static_cast<int16_t>((value_ - 50e3)/390),
            .sign = sign_,
            .horizon = static_cast<uint16_t>(std::round(
                std::log((end_ - start_)/0.0165)/std::log(2.)
            )),
        };
    }

    constexpr Sample(bool universal_sign):
        start_(std::numeric_limits<double>::lowest()),
        end_(std::numeric_limits<double>::max()),
        sign_(universal_sign),
        // If sign is 1, this sample's value is greater than all possible values
        // If sign is 0, this sample's value is lower than all possible values
        value_(universal_sign? 100e3: 0.),
        clear_(false),
        confidence_(0)
    { }

    constexpr Sample(
        double start, double end, bool sign, double value, bool clear, uint8_t confidence
    ):
        start_(start), end_(end), sign_(sign), value_(value), clear_(clear), confidence_(confidence)
    { }

    constexpr double start() const { return start_; }
    constexpr double end() const { return end_; }
    constexpr bool should_clear() const { return clear_; }
    constexpr double value() const { return value_; }
    constexpr bool sign() const { return sign_; }
    constexpr uint8_t confidence() const { return confidence_; }

    constexpr bool conflicts(const Sample &other) const {
        return
            (
                // gt           lt
                sign_ && !other.sign_ &&
                value_ < other.value_
            ) ||
            (
                // lt           gt
                !sign_ && other.sign_ &&
                value_ > other.value_
            );
    }

    constexpr bool is_superset_of(const Sample &other) const {
        return end_ >= other.end_
            && (
                // upper bound superset
                (sign_ && other.sign_ && value_ <= other.value_) ||
                // lower bound superset
                (!sign_ && !other.sign_ && value_ >= other.value_)
            );
    }

    constexpr bool overrides(const Sample &other) const {
        if (conflicts(other)) {
            return confidence_ > other.confidence_
                || (confidence_ == other.confidence_ && start_ < other.start_);
        }
        else return false;
    }

    constexpr const Sample &resolve_lower(const Sample &other) const {
        // Only narrow the range by preferring the greater lower bound
        return value_ > other.value_? *this: other;
    }

    constexpr const Sample &resolve_upper(const Sample &other) const {
        // Only narrow the range by preferring the lesser upper bound
        return value_ < other.value_? *this: other;
    }

private:
    double start_, end_;
    bool sign_;
    double value_;
    bool clear_;
    uint8_t confidence_;
};

constexpr static const Sample universal_lower(false), universal_upper(true);


class Photometer {
public:
    Photometer() = default;

    Photometer(const Photometer &current, double future):
        lower_by_end(
            current.lower_by_end.upper_bound(future),
            current.lower_by_end.cend()
        ),
        upper_by_end(
            current.upper_by_end.upper_bound(future),
            current.upper_by_end.cend()
        )
    { }

    size_t size() const { return lower_by_end.size() + upper_by_end.size(); }

    void consume(
        double now,               // monotonic seconds
        const uint8_t (&data)[2]  // raw from the sensor
    ) {
        consume(Sample::from_raw(now, data));
    }

    void consume(const Sample &sample) {
        if (sample.should_clear()) {
            lower_by_end.clear();
            upper_by_end.clear();
        }
        else erase_old(sample.start());

        MapT &target = sample.sign()? upper_by_end: lower_by_end;

        MapT::const_iterator subset_of = std::find_if(
            target.cbegin(),
            target.cend(),
            [sample](const std::pair<const double, Sample> &other) {
                return other.second.is_superset_of(sample);
            }
        );
        if (subset_of == target.cend())
            target.emplace(sample.end(), sample);
    }

    double lower() const {
        const Sample *effective_lower = &universal_lower;
        for (const std::pair<const double, Sample> &kvl: lower_by_end) {
            bool keep = true;
            // This is O(n2): we can do better, but for now this is simple to implement and validate
            for (const std::pair<const double, Sample> &kvu: upper_by_end) {
                if (kvu.second.overrides(kvl.second)) {
                    keep = false;
                    break;
                }
            }
            if (keep)
                effective_lower = &effective_lower->resolve_lower(kvl.second);
        }
        return effective_lower->value();
    }

    double upper() const {
        const Sample *effective_upper = &universal_upper;
        for (const std::pair<const double, Sample> &kvu: upper_by_end) {
            bool keep = true;
            // This is O(n2): we can do better, but for now this is simple to implement and validate
            for (const std::pair<const double, Sample> &kvl: lower_by_end) {
                if (kvl.second.overrides(kvu.second)) {
                    keep = false;
                    break;
                }
            }
            if (keep)
                effective_upper = &effective_upper->resolve_upper(kvu.second);
        }
        return effective_upper->value();
    }

    // No 'now'; all samples considered current
    double estimate() const {
        return 0.5*(lower() + upper());
    }

    double estimate(
        double now  // monotonic seconds
    ) const {
        Photometer as_of(*this, now);
        return as_of.estimate();
    }

private:
    typedef std::multimap<double, Sample> MapT;

    void erase_old(double now) {
        // Erase samples with ends up to and including now
        // This uses a half-open interval
        lower_by_end.erase(
            lower_by_end.cbegin(),
            lower_by_end.upper_bound(now)
        );
        upper_by_end.erase(
            upper_by_end.cbegin(),
            upper_by_end.upper_bound(now)
        );
    }

    MapT lower_by_end, upper_by_end;
};

constexpr bool is_close(double a, double b, double epsilon = 1e-12) {
    return std::abs(a - b) < epsilon;
}

void test_serialise() {
    std::cout << "test_serialise\n";
    RawSample samp {
        .confidence = 2,
        .clear = 0,
        .value = 0b11110000,
        .sign = 0,
        .horizon = 0b0101,
    };
    const uint8_t *raw = reinterpret_cast<const uint8_t*>(&samp);
    assert(raw[0] == 0b10000010u);
    assert(raw[1] == 0b01010111u);

    samp.dump();
}

void test_deserialise() {
    std::cout << "test_deserialise\n";
    double now = 0.5;
    Sample samp = Sample::from_raw(now, {
        0b10000010u,
        0b01010111u,
    });
    samp.raw().dump();
    assert(is_close(samp.end(), now + 0.0165*std::pow(2., 0b0101u)));
    assert(samp.sign() == false);
    assert(is_close(samp.value(), 50e3 + 390.*static_cast<int8_t>(0b11110000)));
    assert(is_close(samp.value(), 50e3 - 390.*16));
    assert(samp.should_clear() == false);
    assert(samp.confidence() == 0b10u);
}

void test_empty() {
    std::cout << "test_empty\n";
    Photometer meter;
    assert(meter.size() == 0);
    assert(is_close(meter.estimate(0), 50e3));
    assert(is_close(meter.estimate(), 50e3));
}

void test_simple_lower() {
    std::cout << "test_simple_lower\n";
    // Single sample
    Photometer meter;
    Sample samp0(1.1, 1.5, false, 65e3, false, 0);
    meter.consume(samp0);
    samp0.raw().dump();
    assert(meter.size() == 1);
    assert(is_close(meter.estimate(1.2), 82500));

    // Second overriding sample
    Sample samp1(1.2, 1.8, false, 70e3, false, 0);
    meter.consume(samp1);
    samp1.raw().dump();
    assert(meter.size() == 2);
    assert(is_close(meter.estimate(1.3), 85e3));

    // First sample expires
    assert(is_close(meter.estimate(1.6), 85e3));

    // All samples expire
    assert(is_close(meter.estimate(2.0), 50e3));

    // Single sample
    Sample samp2(2.2, 2.5, false, 50e3, false, 0);
    meter.consume(samp2);
    samp2.raw().dump();
    assert(meter.size() == 1);
    assert(is_close(meter.estimate(2.3), 75e3));

    // Clearing sample
    Sample samp3(2.3, 2.5, false, 60e3, true, 0);
    meter.consume(samp3);
    samp3.raw().dump();
    assert(meter.size() == 1);
    assert(is_close(meter.estimate(2.4), 80e3));

    // Sample expires on boundary
    assert(is_close(meter.estimate(2.5), 50e3));
}

void test_simple_upper() {
    std::cout << "test_simple_upper\n";
    // Single sample
    Photometer meter;
    Sample samp0(1.1, 1.5, true, 40e3, false, 0);
    meter.consume(samp0);
    samp0.raw().dump();
    assert(meter.size() == 1);
    assert(is_close(meter.estimate(1.2), 20e3));

    // Second overriding sample
    Sample samp1(1.2, 1.8, true, 30e3, false, 0);
    meter.consume(samp1);
    samp1.raw().dump();
    assert(meter.size() == 2);
    assert(is_close(meter.estimate(1.3), 15e3));

    // First sample expires
    assert(is_close(meter.estimate(1.6), 15e3));

    // All samples expire
    assert(is_close(meter.estimate(2.0), 50e3));

    // Single sample
    Sample samp2(2.2, 2.5, true, 50e3, false, 0);
    meter.consume(samp2);
    samp2.raw().dump();
    assert(meter.size() == 1);
    assert(is_close(meter.estimate(2.3), 25e3));

    // Clearing sample
    Sample samp3(2.3, 2.5, true, 60e3, true, 0);
    meter.consume(samp3);
    samp3.raw().dump();
    assert(meter.size() == 1);
    assert(is_close(meter.estimate(2.4), 30e3));

    // Sample expires on boundary
    assert(is_close(meter.estimate(2.5), 50e3));
}

void test_superset() {
    std::cout << "test_superset\n";
    Photometer meter;

    Sample sampl(1.1, 1.5, false, 0, false, 0);
    meter.consume(sampl);
    sampl.raw().dump();

    Sample samp0(1.1, 1.5, true, 40e3, false, 0);
    meter.consume(samp0);
    samp0.raw().dump();

    Sample samp1(1.2, 1.4, true, 45e3, false, 0);
    meter.consume(samp1);
    samp1.raw().dump();

    assert(meter.size() == 2);
    assert(is_close(meter.estimate(1.3), 20e3));
}

void test_double_bound() {
    std::cout << "test_double_bound\n";
    Photometer meter;
    Sample samp0(1.0, 1.5, false, 20e3, false, 0);
    meter.consume(samp0);
    samp0.raw().dump();
    Sample samp1(1.0, 1.5, true, 40e3, false, 0);
    meter.consume(samp1);
    samp1.raw().dump();
    assert(meter.size() == 2);
    assert(is_close(meter.estimate(1.1), 30e3));
}

void test_override_confidence() {
    std::cout << "test_override_confidence\n";
    Photometer meter;
    // Second one wins
    Sample samp0(1.0, 2.0, false, 40e3, false, 0);
    meter.consume(samp0);
    samp0.raw().dump();
    Sample samp1(1.0, 1.5, true, 20e3, false, 1);
    meter.consume(samp1);
    samp1.raw().dump();
    assert(meter.size() == 2);
    assert(is_close(meter.estimate(1.2), 10e3));
    assert(is_close(meter.estimate(1.7), 70e3));

    // First one wins
    Sample samp2(3.0, 4.0, false, 40e3, false, 1);
    meter.consume(samp2);
    samp2.raw().dump();
    Sample samp3(3.0, 3.5, true, 20e3, false, 0);
    meter.consume(samp3);
    samp3.raw().dump();
    assert(meter.size() == 2);
    assert(is_close(meter.estimate(3.2), 70e3));
    assert(is_close(meter.estimate(3.7), 70e3));
}

void test_override_time() {
    std::cout << "test_override_time\n";
    Photometer meter;
    // First one wins
    Sample samp0(1.0, 2.0, false, 60e3, false, 2);
    meter.consume(samp0);
    samp0.raw().dump();
    Sample samp1(1.1, 2.0, true, 30e3, false, 2);
    meter.consume(samp1);
    samp1.raw().dump();
    assert(meter.size() == 2);
    assert(is_close(meter.estimate(1.2), 80e3));
}

// Public tests

void passert(const char *desc, double value, double lower, double nom, double upper) {
    if (value < lower)
        std::cerr << "  " << desc << " failed: " << value << " < " << lower << '\n';
    else if (value > upper)
        std::cerr << "  " << desc << " failed: " << value << " > " << upper << '\n';
    else if (!is_close(value, nom))
        std::cerr << "  " << desc << " warn: " << value << " ~ " << nom << '\n';
    else
        std::cout << "  " << desc << " passed\n";
}

void ptest_empty() {
    std::cout << "ptest_empty\n";
    Photometer meter;
    passert("empty default", meter.estimate(0), 0, 50e3, 100e3);
}

void ptest_simple_lower() {
    std::cout << "ptest_simple_lower\n";
    // Single sample
    Photometer meter;
    meter.consume(1.1, (const uint8_t[2]){ 0x30u, 0x51u });  // conf=0 clear=0 value=64820 sign=0 horizon=0.528
    passert("single sample", meter.estimate(1.2), 64820, (64820 + 100000)*0.5, 100000);

    // Second overriding sample
    meter.consume(1.2, (const uint8_t[2]){ 0x98u, 0x51u });  // conf=0 clear=0 value=69890 sign=0 horizon=0.528
    passert("second overriding sample", meter.estimate(1.3), 69890, (69890 + 100000)*0.5, 100000);

    // First sample expires
    passert("first sample expires", meter.estimate(1.1 + 0.528 + 0.01), 69890, (69890 + 100000)*0.5, 100000);

    // All samples expire
    passert("all samples expire", meter.estimate(1.2 + 0.528 + 0.01), 0, 50e3, 100e3);

    // Single sample
    meter.consume(2.20, (const uint8_t[2]){ 0x0u, 0xf0u });  // conf=0 clear=0 value=50000 sign=0 horizon=540.672
    passert("single pre-clear sample", meter.estimate(2.205), 50e3, 75e3, 100e3);

    // Clearing sample
    meter.consume(2.21, (const uint8_t[2]){ 0xccu, 0x40u });  // conf=0 clear=1 value=59750 sign=0 horizon=0.264
    passert("clearing sample", meter.estimate(2.4), 59750, (59750 + 100000)*0.5, 100e3);

    // Sample expires on boundary
    passert("before boundary", meter.estimate(2.21 + 0.264 - 1e-4), 59750, (59750 + 100000)*0.5, 100e3);
    passert("sample expires on boundary", meter.estimate(2.21 + 0.264), 0, 50e3, 100e3);
}

void ptest_simple_upper() {
    std::cout << "ptest_simple_upper\n";
    // Single sample
    Photometer meter;
    meter.consume(1.1, (const uint8_t[2]){ 0x38u, 0x5fu });  // conf=0 clear=0 value=40250 sign=1 horizon=0.528
    passert("single sample", meter.estimate(1.2), 0, 0.5*40250, 40250);

    // Second overriding sample
    meter.consume(1.2, (const uint8_t[2]){ 0x68u, 0x5eu });  // conf=0 clear=0 value=30110 sign=1 horizon=0.528
    passert("second overriding sample", meter.estimate(1.3), 0, 0.5*30110, 30110);

    // First sample expires
    passert("first sample expires", meter.estimate(1.1 + 0.528 + 0.01), 0, 0.5*30110, 30110);

    // All samples expire
    passert("all samples expire", meter.estimate(1.2 + 0.528 + 0.01), 0, 50e3, 100e3);

    // Single sample
    meter.consume(2.20, (const uint8_t[2]){ 0x0u, 0xf8u });  // conf=0 clear=0 value=50000 sign=1 horizon=540.672
    passert("single pre-clear sample", meter.estimate(2.205), 0, 0.5*50e3, 50e3);

    // Clearing sample
    meter.consume(2.21, (const uint8_t[2]){ 0xccu, 0x48u });  // conf=0 clear=1 value=59750 sign=1 horizon=0.264
    passert("clearing sample", meter.estimate(2.4), 0, 0.5*59750, 59750);

    // Sample expires on boundary
    passert("before boundary", meter.estimate(2.21 + 0.264 - 1e-4), 0, 0.5*59750, 59750);
    passert("sample expires on boundary", meter.estimate(2.21 + 0.264), 0, 50e3, 100e3);
}

void ptest_superset() {
    std::cout << "ptest_superset\n";
    Photometer meter;
    meter.consume(1.1, (const uint8_t[2]){ 0x0u, 0x54u });  // conf=0 clear=0 value=80 sign=0 horizon=0.528
    meter.consume(1.1, (const uint8_t[2]){ 0x38u, 0x5fu });  // conf=0 clear=0 value=40250 sign=1 horizon=0.528
    meter.consume(1.2, (const uint8_t[2]){ 0xa0u, 0x4fu });  // conf=0 clear=0 value=45320 sign=1 horizon=0.264
    passert("superset", meter.estimate(1.3), 80, 0.5*(80 + 40250), 40250);
}

void ptest_double_bound() {
    std::cout << "ptest_double_bound\n";
    Photometer meter;
    meter.consume(10e3, (const uint8_t[2]){ 0xa0u, 0x55u });  // conf=0 clear=0 value=20360 sign=0 horizon=0.528
    meter.consume(10e3, (const uint8_t[2]){ 0x38u, 0x5fu });  // conf=0 clear=0 value=40250 sign=1 horizon=0.528
    passert("double bound", meter.estimate(10000.1), 20360, (20360 + 40250)*0.5, 40250);
}

void ptest_override_confidence() {
    std::cout << "ptest_override_confidence\n";
    Photometer meter;
    // Second one wins
    meter.consume(1.0, (const uint8_t[2]){ 0x38u, 0x67u });  // conf=0 clear=0 value=40250 sign=0 horizon=1.056
    meter.consume(1.0, (const uint8_t[2]){ 0xa1u, 0x5du });  // conf=1 clear=0 value=20360 sign=1 horizon=0.528
    passert("second one wins - before expiry", meter.estimate(1.2), 0, 0.5*20360, 20360);
    passert("second one loses - after expiry", meter.estimate(1.7), 40250, (40250 + 100e3)*0.5, 100e3);

    // First one wins
    meter.consume(3.0, (const uint8_t[2]){ 0x39u, 0x67u });  // conf=1 clear=0 value=40250 sign=0 horizon=1.056
    meter.consume(3.0, (const uint8_t[2]){ 0xa0u, 0x5du });  // conf=0 clear=0 value=20360 sign=1 horizon=0.528
    passert("first one wins - before expiry", meter.estimate(3.2), 40250, (40250 + 100e3)*0.5, 100e3);
    passert("first one still wins after expiry", meter.estimate(3.7), 40250, (40250 + 100e3)*0.5, 100e3);
}

void ptest_override_time() {
    std::cout << "ptest_override_time\n";
    Photometer meter;
    // First one wins
    meter.consume(-1000, (const uint8_t[2]){ 0xcau, 0x60u });  // conf=2 clear=0 value=59750 sign=0 horizon=1.056
    meter.consume(-999.9, (const uint8_t[2]){ 0x6au, 0x6eu });  // conf=2 clear=0 value=30110 sign=1 horizon=1.056
    passert("first time wins", meter.estimate(-999.8), 59750, (59750 + 100e3)*0.5, 100e3);
}

void test_reference() {
    test_serialise();
    test_deserialise();
    test_empty();
    test_simple_lower();
    test_simple_upper();
    test_superset();
    test_double_bound();
    test_override_confidence();
    test_override_time();
}

void test_public() {
    ptest_empty();
    ptest_simple_lower();
    ptest_simple_upper();
    ptest_superset();
    ptest_double_bound();
    ptest_override_confidence();
    ptest_override_time();
}

}


int main() {
    test_reference();
    test_public();
    return 0;
}
