
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>

class MovingMedianFilter
{
public:
    MovingMedianFilter() {
        double resetValue = 0;
        int windowSize=5;
        setFilterParameters(windowSize, resetValue);
    }

    explicit MovingMedianFilter(int windowSize, const double &resetValue)
    {   
        setFilterParameters(windowSize, resetValue);
    }

    void reset(const double &y_0)
    {
        index_ = 0;
        std::fill(samples_.begin(), samples_.end(), y_0);
        workVector_ = samples_;
    }

    void setFilterParameters(int windowSize, const double &resetValue)
    {
        windowSize_ = windowSize;
        numSamples_ = windowSize_ * 2 + 1;
        std::cout<<"---------- num of samples ------ \n "<<numSamples_<<std::endl;
        samples_.resize(numSamples_);
        workVector_.resize(numSamples_);
        reset(resetValue);
    }
    double advance(double newValue)
    {
        samples_[index_] = newValue;
        index_++;
        if (index_ >= numSamples_)
        {
            index_ = 0;
        }

        workVector_ = samples_;
        std::nth_element(workVector_.begin(), workVector_.begin() +windowSize_, workVector_.end());

        return getFilteredValue();
    }

    double getFilteredValue() const { return workVector_[windowSize_]; }

private:
    int windowSize_;
    int numSamples_;
    int index_;
    //! The vector of last few samples to compute the median for.
    std::vector<double> samples_;
    //! The partially sorted version of the last few samples to find and store the median.
    std::vector<double> workVector_;
};

//! EIGEN VECTOR FILTER !//

// template <typename ValueType_>
// class FilterMovingMedian_VECT<ValueType_, typename std::enable_if<traits::is_eigen_matrix<ValueType_>::value>::type>
// {
//     using scalar_t = typename Eigen::internal::traits<ValueType_>::Scalar;

// public:
//     //! @brief Default constructor. Constructs a filter that just returns the latest measured value.
//     FilterMovingMedian_VECT() : FilterMovingMedian_VECT(0) {}

//     /**
//      * @brief Construct a filter with given window and initialize with a given value.
//      * @param windowSize : single sided size of the filter window. The double sided window will contain 2 * windowSize + 1 values.
//      * @param resetValue : value to initialize the filter with.
//      */
//     explicit FilterMovingMedian_VECT(int windowSize, const ValueType_ &resetValue = getDefaultValue<ValueType_>())
//     {
//         setFilterParameters(windowSize, resetValue);
//     }

//     /**
//      * @brief Reset the filter with given window and initialize with a given value.
//      * @param windowSize : single sided size of the filter window. The double sided window will contain 2 * windowSize + 1 values.
//      * @param resetValue : value to initialize the filter with.
//      */
//     void setFilterParameters(int windowSize, const ValueType_ &resetValue = getDefaultValue<ValueType_>())
//     {
//         const auto numberOfChannels = resetValue.size();
//         filters_.clear();
//         filters_.reserve(numberOfChannels);
//         for (size_t i = 0; i < numberOfChannels; ++i)
//         {
//             filters_.emplace_back(FilterMovingMedian<scalar_t>(windowSize, resetValue[i]));
//         }
//     }

//     /**
//      * @brief Reset the filter with a given value. The full filter window will be filled with that value.
//      * @param y_0 : value to reset the filter to.
//      */
//     void reset(const ValueType_ &y_0 = getDefaultValue<ValueType_>())
//     {
//         for (size_t i = 0; i < filters_.size(); ++i)
//         {
//             filters_[i].reset(y_0[i]);
//         }
//     }

//     /**
//      * @brief Advance the filter
//      * @param newValue : new measurement
//      * @return new filtered value (taking into account the new measurement)
//      */
//     ValueType_ advance(const ValueType_ &newValue)
//     {
//         ValueType_ results;
//         results.resize(filters_.size());
//         for (size_t i = 0; i < filters_.size(); ++i)
//         {
//             results[i] = filters_[i].advance(newValue[i]);
//         }
//         return results;
//     }

//     /**
//      * @return the latest filtered value
//      */
//     ValueType_ getFilteredValue() const
//     {
//         ValueType_ results;
//         results.resize(filters_.size());
//         for (size_t i = 0; i < filters_.size(); ++i)
//         {
//             results[i] = filters_[i].getFilteredValue();
//         }
//         return results;
//     }

// private:
//     //! Filter implementation per dimensions of the underlying type.
//     std::vector<FilterMovingMedian<scalar_t>> filters_;
// };
