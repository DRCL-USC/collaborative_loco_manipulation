
#include <ocs2_core/Types.h>

namespace ocs2
{
    namespace object_manipulation
    {
        template <typename T>
        class LowPassFilter
        {
        public:
            LowPassFilter(double alpha) : alpha(alpha), initialized(false) {}

            T process(T input)
            {
                if (!initialized)
                {
                    prev_output = input;
                    initialized = true;
                }
                else
                {
                    prev_output = alpha * input + (1.0 - alpha) * prev_output;
                }
                return prev_output;
            }

        private:
            scalar_t alpha;       // Smoothing factor (0 < alpha <= 1)
            bool initialized;   // Indicates if the filter has been initialized
            T prev_output; // Previous output value
        };

        void saturate(vector_t &vec, vector_t min_val, vector_t max_val)
            {
                for (int i = 0; i < vec.size(); ++i)
                {
                    vec(i) = std::min(std::max(vec(i), min_val(i)), max_val(i));
                }
            }

    } // namespace object_manipulation
} // namespace ocs2
