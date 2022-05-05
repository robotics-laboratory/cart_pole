#include "globals.h"

#include <iomanip>
#include <sstream>
#include <unordered_map>

#include "stepper.h"

// namespace {
    const int DEBUG_LED_PIN = 2;

std::ostream &operator<<(std::ostream &out, Error error) { return out << static_cast<int>(error); }

std::istream &operator>>(std::istream &in, Error &error) {
    int err = 0;
    in >> err;
    error = static_cast<Error>(err);
    return in;
}

struct FieldBase {
    virtual ~FieldBase() = default;
    virtual std::string Format() const;
    virtual void Prepare(const std::string &text);
    virtual void Commit();
    virtual void Reset();
};

template <typename T>
std::string formatField(T value) {
    std::stringstream stream;
    stream << value;
    return stream.str();
}

template <>
std::string formatField<>(float value) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(5) << value;
    return stream.str();
}

template <>
std::string formatField<>(bool value) {
    return value ? "true" : "false";
}

template <typename T>
T parseField(const std::string &text) {
    std::stringstream stream(text);
    T value;
    stream >> value;
    if (stream.fail()) {
        throw std::runtime_error{"Failed to parse value"};
    }
    return value;
}

template <>
bool parseField<>(const std::string &text) {
    if (text == "true") {
        return true;
    } else if (text == "false") {
        return false;
    }
    throw std::runtime_error("Failed to parse bool");
}

template <typename T, FieldID F>
T validateField(T value) {
    return value;
}

float validateFloatRange(float value, float min, float max, bool clamp = false) {
    if (std::isinf(value)) {
        throw std::runtime_error{"Infinite values are not allowed"};
    }
    if (std::isnan(value)) {
        throw std::runtime_error{"NaN is not allowed"};
    }
    if (value < min) {
        if (clamp) {
            return min;
        }
        std::stringstream stream;
        stream << std::fixed << std::setprecision(5) << "Out of range: " << value << " < " << min;
        throw std::runtime_error{stream.str()};
    }
    if (max < value) {
        if (clamp) {
            return max;
        }
        std::stringstream stream;
        stream << std::fixed << std::setprecision(5) << "Out of range: " << value << " > " << max;
        throw std::runtime_error{stream.str()};
    }
    return value;
}

template <>
float validateField<float, FieldID::max_x>(float value) {
    Globals &G = GetGlobals();
    return validateFloatRange(value, 0, G.hw_max_x);
}

template <>
float validateField<float, FieldID::max_v>(float value) {
    Globals &G = GetGlobals();
    return validateFloatRange(value, 0, G.hw_max_v);
}

template <>
float validateField<float, FieldID::max_a>(float value) {
    Globals &G = GetGlobals();
    return validateFloatRange(value, 0, G.hw_max_a);
}

template <>
float validateField<float, FieldID::trgt_x>(float value) {
    Globals &G = GetGlobals();
    try {
        return validateFloatRange(value, -G.max_x, G.max_x, G.clamp_x);
    } catch (std::runtime_error &e) {
        G.errcode = Error::X_OVERFLOW;
        throw;
    }
}

template <>
float validateField<float, FieldID::trgt_v>(float value) {
    Globals &G = GetGlobals();
    try {
        return validateFloatRange(value, -G.max_v, G.max_v, G.clamp_v);
    } catch (std::runtime_error &e) {
        G.errcode = Error::V_OVERFLOW;
        throw;
    }
}

template <>
float validateField<float, FieldID::trgt_a>(float value) {
    Globals &G = GetGlobals();
    try {
        return validateFloatRange(value, -G.max_a, G.max_a, G.clamp_a);
    } catch (std::runtime_error &e) {
        G.errcode = Error::A_OVERFLOW;
        throw;
    }
}

template <typename T, FieldID F>
void updateField(T value) {
    // do nothing
}

template <>
void updateField<float, FieldID::max_v>(float value) {
    Stepper &S = GetStepper();
    S.SetSpeed(value);
}

template <>
void updateField<float, FieldID::max_a>(float value) {
    Stepper &S = GetStepper();
    S.SetAcceleration(value);
}

template <>
void updateField<float, FieldID::trgt_x>(float value) {
    Stepper &S = GetStepper();
    S.SetTargetPosition(value);
}

template <>
void updateField<float, FieldID::trgt_v>(float value) {
    // Stepper &S = GetStepper();
    // S.SetTargetVelocity(value);
}

template <>
void updateField<float, FieldID::trgt_a>(float value) {
    Stepper &S = GetStepper();
    S.SetTargetAcceleration(value);
}

template <>
void updateField<bool, FieldID::debug_led>(bool value) {
    pinMode(DEBUG_LED_PIN, OUTPUT);
    digitalWrite(DEBUG_LED_PIN, value);
}

template <typename T, FieldID F, bool readonly = false>
class Field : public FieldBase {
    T &global_value;
    T pending_value;
    const T default_value;

public:
    Field(T &global_value)
        : global_value(global_value),
          pending_value(),
          default_value(global_value) {}

    std::string Format() const override { return formatField<T>(global_value); }

    void Prepare(const std::string &text) override {
        if (readonly) {
            throw std::runtime_error{"Readonly violation"};
        }
        pending_value = validateField<T, F>(parseField<T>(text));
    }

    void Commit() override {
        global_value = pending_value;
        updateField<T, F>(global_value);
    }

    void Reset() override {
        global_value = default_value;
        updateField<T, F>(global_value);
    }
};


template <typename T, FieldID F, bool readonly>
std::unique_ptr<Field<T, F, readonly>> makeField(T &value) {
    return std::unique_ptr<Field<T, F, readonly>>(new Field<T, F, readonly>(value));
}

using FieldMap = std::unordered_map<std::string, std::unique_ptr<FieldBase>>;

FieldMap constructConfigFieldMap() {
    Globals &G = GetGlobals();
    FieldMap map;
    map["max_x"] = makeField<float, FieldID::max_x, false>(G.max_x);
    map["max_v"] = makeField<float, FieldID::max_v, false>(G.max_v);
    map["max_a"] = makeField<float, FieldID::max_a, false>(G.max_a);
    map["hw_max_x"] = makeField<float, FieldID::hw_max_x, true>(G.hw_max_x);
    map["hw_max_v"] = makeField<float, FieldID::hw_max_v, true>(G.hw_max_v);
    map["hw_max_a"] = makeField<float, FieldID::hw_max_a, true>(G.hw_max_a);
    map["clamp_x"] = makeField<bool, FieldID::clamp_x, false>(G.clamp_x);
    map["clamp_v"] = makeField<bool, FieldID::clamp_v, false>(G.clamp_v);
    map["clamp_a"] = makeField<bool, FieldID::clamp_a, false>(G.clamp_a);
    map["debug_led"] = makeField<bool, FieldID::debug_led, false>(G.clamp_a);
    return map;
}

FieldMap constructStateFieldMap() {
    Globals &G = GetGlobals();
    FieldMap map;
    map["curr_x"] = makeField<float, FieldID::curr_x, true>(G.curr_x);
    map["curr_v"] = makeField<float, FieldID::curr_v, true>(G.curr_v);
    map["curr_a"] = makeField<float, FieldID::curr_a, true>(G.curr_a);
    map["pole_x"] = makeField<float, FieldID::pole_x, true>(G.pole_x);
    map["pole_v"] = makeField<float, FieldID::pole_v, true>(G.pole_v);
    map["errcode"] = makeField<Error, FieldID::errcode, true>(G.errcode);
    map["imu_a"] = makeField<float, FieldID::imu_a, true>(G.imu_a);
    map["motor_x"] = makeField<float, FieldID::motor_x, true>(G.motor_x);
    map["motor_v"] = makeField<float, FieldID::motor_v, true>(G.motor_v);
    return map;
}

FieldMap constructTargetFieldMap() {
    Globals &G = GetGlobals();
    FieldMap map;
    map["trgt_x"] = makeField<float, FieldID::trgt_x, false>(G.trgt_x);
    map["trgt_v"] = makeField<float, FieldID::trgt_v, false>(G.trgt_v);
    map["trgt_a"] = makeField<float, FieldID::trgt_a, false>(G.trgt_a);
    return map;
}

FieldMap CONFIG_FIELDS = constructConfigFieldMap();
FieldMap STATE_FIELDS = constructStateFieldMap();
FieldMap TARGET_FIELDS = constructTargetFieldMap();

FieldMap &lookupFieldMap(const std::string &group) {
    if (group == "config") {
        return CONFIG_FIELDS;
    } else if (group == "state") {
        return STATE_FIELDS;
    } else if (group == "target") {
        return TARGET_FIELDS;
    }
    throw std::runtime_error{"Unknown group"};
}

void resetFieldMap(FieldMap &map) {
    for (auto &&kv : map) {
        kv.second->Reset();
    }
}

FieldBase *getField(const FieldMap &map, const std::string &key) {
    auto it = map.find(key);
    if (it == map.end()) {
        std::stringstream stream;
        stream << "Unknown key: " << key;
        throw std::runtime_error{stream.str()};
    }
    return it->second.get();
}
// }  // namespace

std::string Globals::Get(const std::string &group, const std::string &key) const {
    const FieldMap &map = lookupFieldMap(group);
    return getField(map, key)->Format();
}

std::vector<std::pair<std::string, std::string>> Globals::Get(const std::string &group) const {
    const FieldMap &map = lookupFieldMap(group);
    std::vector<std::pair<std::string, std::string>> res;
    for (auto &&kv : map) {
        res.emplace_back(kv.first, kv.second->Format());
    }
    return res;
}

void Globals::Prepare(const std::string &group, const std::string &key, const std::string &value) {
    // FieldMap &map = lookupFieldMap(group);
    // getField(map, key)->Prepare(value);
}

void Globals::Commit(const std::string &group, const std::string &key) {
    FieldMap &map = lookupFieldMap(group);
    getField(map, key)->Commit();
}

void Globals::Reset() {
    resetFieldMap(CONFIG_FIELDS);
    resetFieldMap(STATE_FIELDS);
    resetFieldMap(TARGET_FIELDS);
}

Globals &GetGlobals() {
    static Globals globals{
        /* CONFIG */
        0.0,                // [m] Absolute max cart position
        0.5,                // [m/s] Absolute max cart velocity
        1.0,                // [m/s^2] Absolute max cart acceleration
        0.0,                // [m] Absolute max hardware-allowed position
        10,                 // fixme  // [m/s] Absolute max hardware-allowed velocity
        10,                 // fixme  // [m/s^2] Absolute max hardware-allowed acceleration
        false,              // Clamp X to allowed range instead of raising error
        false,              // Clamp V to allowed range instead of raising error
        false,              // Clamp A to allowed range instead of raising error
        false,
        /* STATE */
        0.0,                // [m] Current cart position
        0.0,                // [m/s] Current cart velocity
        0.0,                // [m/s^2] Current cart acceleration
        0.0,                // [rad] Current pole angle
        0.0,                // [rad/s] Current pole angular velocity
        Error::NEED_RESET,  // Current error code
        0.0,
        0.0,
        0.0,
        /* TARGET */
        0.0,                // [m] Target cart position
        0.0,                // [m/s] Target cart velocity
        0.0,                // [m/s^2] Target cart acceleration
        /* MISC */
        0.0,        // [m] Total Length as determined during homing
    };
    return globals;
}
