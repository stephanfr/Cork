
#include <catch2/catch_all.hpp>

#include "util/ConstuctOnceResizeableVector.h"

constexpr int   DEFAULT_NUMBER_OF_ELEMENTS = 10;
constexpr int   RESIZE_VALUE = 5;
constexpr int   REALLY_BIG_SIZE = 123456;

//	Resettable element for testing

class TestElement : public SEFUtility::Resettable
{
   public:
    TestElement() : m_value(0), m_reset(true) {}

    virtual ~TestElement() {}

    void reset()
    {
        m_value = 0;
        m_reset = true;
    }

    bool isReset() const { return (m_reset); }

    void setValue(long newValue)
    {
        m_value = newValue;
        m_reset = false;
    }

    long getValue() const { return (m_value); }

   private:
    long m_value;
    bool m_reset;
};

//	Run a variety of checks on the size of the vector and insure we cannot overrun vector at its end.

void CheckSize(SEFUtility::ConstructOnceResizeableVector<TestElement>& testVector, size_t correctSize)
{
    REQUIRE(testVector.size() == correctSize);

    size_t numElements = 0;

    for (auto& element : testVector)
    {
        numElements++;
    }

    REQUIRE(numElements == correctSize);

    numElements = 0;
    long runningSum = 0;

    for (size_t i = 0; i < testVector.size(); i++)
    {
        numElements++;
        runningSum += testVector[i].getValue();
    }

    REQUIRE(numElements == correctSize);

    numElements = 0;

    for (const auto& element : testVector)
    {
        numElements++;
    }

    REQUIRE(numElements == correctSize);

    numElements = 0;

    for (SEFUtility::ConstructOnceResizeableVector<TestElement>::iterator itr = testVector.begin();
         itr != testVector.end(); itr++)
    {
        numElements++;
    }

    REQUIRE(numElements == correctSize);

    numElements = 0;

    for (SEFUtility::ConstructOnceResizeableVector<TestElement>::const_iterator itr = testVector.begin();
         itr != testVector.end(); itr++)
    {
        numElements++;
    }

    REQUIRE(numElements == correctSize);

    SEFUtility::ConstructOnceResizeableVector<TestElement>::iterator itr = testVector.end();

    REQUIRE(itr == testVector.end());
    itr++;
    REQUIRE(itr++ == testVector.end());
    REQUIRE(itr++ == testVector.end());

    SEFUtility::ConstructOnceResizeableVector<TestElement>::iterator citr = testVector.end();

    REQUIRE(citr == testVector.end());
    citr++;
    REQUIRE(citr++ == testVector.end());
    REQUIRE(citr++ == testVector.end());
}

//	Insure all elements of the vector are reset

bool CheckAllReset(const SEFUtility::ConstructOnceResizeableVector<TestElement>& testVector)
{
    for (const auto& element : testVector)
    {
        if (!element.isReset())
        {
            return (false);
        }
    }

    return (true);
}

//	Insure none of the elements of the vector are reset

bool CheckAllNotReset(const SEFUtility::ConstructOnceResizeableVector<TestElement>& testVector)
{
    for (const auto& element : testVector)
    {
        if (element.isReset())
        {
            return (false);
        }
    }

    return (true);
}

//	Compute a sum of all values in the vector

long ComputeRunningSum(const SEFUtility::ConstructOnceResizeableVector<TestElement>& testVector)
{
    long runningSum = 0;

    for (const auto& element : testVector)
    {
        runningSum += element.getValue();
    }

    long secondSum = 0;

    for (size_t i = 0; i < testVector.size(); i++)
    {
        secondSum += testVector[i].getValue();
    }

    REQUIRE(secondSum == runningSum);

    return (runningSum);
}

//	Run a collection of tests to insure the vector runs correctly

TEST_CASE("Construct Once Resizeable Vector Test", "[cork-base]")
{
    //	Construct the vector of ten elements and insure it is correctly sized at zero initially

    SEFUtility::ConstructOnceResizeableVector<TestElement> testVector(DEFAULT_NUMBER_OF_ELEMENTS);

    REQUIRE(testVector.size() == 0);

    REQUIRE(testVector.reservedSize() == DEFAULT_NUMBER_OF_ELEMENTS);

    //	Resize the vector to RESIZE_VALUE - insure it works correctly

    testVector.resize(RESIZE_VALUE);

    REQUIRE(CheckAllReset(testVector));

    CheckSize(testVector, RESIZE_VALUE);

    long i = 0;
    long runningSum = 0;

    for (auto& element : testVector)
    {
        runningSum += i;
        element.setValue(i++);
    }

    REQUIRE(CheckAllNotReset(testVector));
    REQUIRE(ComputeRunningSum(testVector) == runningSum);

    //	Reset the vector, it shoud be at size zero after it is reset

    testVector.reset();

    CheckSize(testVector, 0);

    //	Reset the vector to a size of RESIZE_VALUE + 1

    testVector.resize(RESIZE_VALUE + 1);

    CheckSize(testVector, 6);

    REQUIRE(CheckAllReset(RESIZE_VALUE + 1));

    i = 0;
    runningSum = 0;

    for (auto& element : testVector)
    {
        runningSum += i;
        element.setValue(i++);
    }

    REQUIRE(CheckAllNotReset(testVector));
    REQUIRE(ComputeRunningSum(testVector) == runningSum);

    //	Directly resize the vector to a size of RESIZE_VALUE - 1

    testVector.resize(RESIZE_VALUE - 1);

    CheckSize(testVector, RESIZE_VALUE - 1);

    REQUIRE(CheckAllReset(testVector));

    //	Resize to a large vector and test

    testVector.resize(REALLY_BIG_SIZE);

    CheckSize(testVector, REALLY_BIG_SIZE);

    REQUIRE(CheckAllReset(testVector));

    i = 0;
    runningSum = 0;

    for (auto& element : testVector)
    {
        runningSum += i;
        element.setValue(i++);
    }

    REQUIRE(CheckAllNotReset(testVector));
    REQUIRE(ComputeRunningSum(testVector) == runningSum);
}
