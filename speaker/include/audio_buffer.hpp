#ifndef AUDIO_BUFFER_HPP
#define AUDIO_BUFFER_HPP

#include <stddef.h>
#include <stdint.h>

namespace AUDIO {
class AudioBuffer {
public:
    AudioBuffer(const uint8_t* data, size_t length)
    : data(data), length(length), position(0) {}

    // Fetch the next audio sample and advance the position
    bool getNextSample(uint8_t& sample) {
        if (position < length) {
            sample = data[position++];
            return true;  // Sample fetched successfully
        }
        return false;  // No more samples
    }

    // Reset the playback position to the beginning of the buffer
    void reset() {
        position = 0;
    }

private:
    const uint8_t* data;  // Pointer to the audio data
    size_t length;        // Length of the audio data
    size_t position;      // Current playback position
};

} // namespace AUDIO

#endif // AUDIO_BUFFER_HPP