
#include <vector>
#include <deque>

class rollingAverage
{
public:
    rollingAverage(const unsigned int& sizeOfRollingAverage, const unsigned int numberOfStates);
    ~rollingAverage(void);

    int calculateRollingAverage(const unsigned int& nextInput);

private:
    std::deque<int> rollingAverageArray;
    std::vector<int> occurrenceOfEachState;
};
