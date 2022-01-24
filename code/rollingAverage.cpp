
#include "rollingAverage.h"

rollingAverage::rollingAverage(const unsigned int& sizeOfRollingAverage, const unsigned int numberOfStates)
{
    for (unsigned int i = 0; i < sizeOfRollingAverage; i++)
		rollingAverageArray.push_back(0);

    occurrenceOfEachState.push_back(sizeOfRollingAverage);
    for (unsigned int i = 1; i < numberOfStates; i++)
        occurrenceOfEachState.push_back(0);
}

rollingAverage::~rollingAverage(void) { }

int rollingAverage::calculateRollingAverage(const unsigned int& nextInput)
{
    occurrenceOfEachState[rollingAverageArray.back()]--;
    rollingAverageArray.pop_back();
    rollingAverageArray.push_front(nextInput);
    occurrenceOfEachState[nextInput]++;

    int mostFrequentState = 0;
    for (unsigned int i = 1; i < occurrenceOfEachState.size(); i++)
    {
        if (occurrenceOfEachState[i] > occurrenceOfEachState[mostFrequentState])
            mostFrequentState = i;
    }
    return mostFrequentState;
}
