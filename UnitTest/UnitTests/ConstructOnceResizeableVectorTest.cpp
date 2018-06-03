#include "pch.h"

#include "ConstuctOnceResizeableVector.h"


//	Resettable element for testing

class TestElement : public SEFUtility::Resettable
{
public :

	TestElement()
		: m_value(0),
		  m_reset( true )
	{}


	virtual ~TestElement()
	{}


	void	reset()
	{
		m_value = 0;
		m_reset = true;
	}

	bool	isReset() const
	{
		return(m_reset);
	}


	void	setValue(long		newValue)
	{
		m_value = newValue;
		m_reset = false;
	}


	long	getValue() const
	{
		return(m_value);
	}

private :

	long		m_value;
	bool		m_reset;

};


//	Run a variety of checks on the size of the vector and insure we cannot overrun vector at its end.

void	CheckSize( SEFUtility::ConstructOnceResizeableVector<TestElement>&		testVector,
				   size_t														correctSize)
{
	EXPECT_EQ(correctSize, testVector.size());

	size_t	numElements = 0;

	for (auto& element : testVector)
	{
		numElements++;
	}

	EXPECT_EQ(correctSize, numElements);

	numElements = 0;
	long	runningSum = 0;

	for ( size_t i = 0; i < testVector.size(); i++ )
	{
		numElements++;
		runningSum += testVector[i].getValue();
	}

	EXPECT_EQ(correctSize, numElements);

	numElements = 0;

	for (const auto& element : testVector)
	{
		numElements++;
	}

	EXPECT_EQ(correctSize, numElements);

	numElements = 0;

	for (SEFUtility::ConstructOnceResizeableVector<TestElement>::iterator itr = testVector.begin(); itr != testVector.end(); itr++)
	{
		numElements++;
	}

	EXPECT_EQ(correctSize, numElements);

	numElements = 0;

	for (SEFUtility::ConstructOnceResizeableVector<TestElement>::const_iterator itr = testVector.begin(); itr != testVector.end(); itr++)
	{
		numElements++;
	}

	EXPECT_EQ(correctSize, numElements);

	SEFUtility::ConstructOnceResizeableVector<TestElement>::iterator itr = testVector.end();

	EXPECT_EQ(testVector.end(), itr);
	itr++;
	EXPECT_EQ(testVector.end(), itr++);
	EXPECT_EQ(testVector.end(), itr++);

	SEFUtility::ConstructOnceResizeableVector<TestElement>::iterator citr = testVector.end();

	EXPECT_EQ(testVector.end(), citr);
	itr++;
	EXPECT_EQ(testVector.end(), citr++);
	EXPECT_EQ(testVector.end(), citr++);
}


//	Insure all elements of the vector are reset

bool	CheckAllReset( const SEFUtility::ConstructOnceResizeableVector<TestElement>&	testVector )
{
	for (const auto& element : testVector)
	{
		if (!element.isReset())
		{
			return(false);
		}
	}

	return(true);
}


//	Insure none of the elements of the vector are reset

bool	CheckAllNotReset(const SEFUtility::ConstructOnceResizeableVector<TestElement>&	testVector)
{
	for (const auto& element : testVector)
	{
		if (element.isReset())
		{
			return(false);
		}
	}

	return(true);
}


//	Compute a sum of all values in the vector

long	ComputeRunningSum(const SEFUtility::ConstructOnceResizeableVector<TestElement>&	testVector)
{
	long		runningSum = 0;

	for (const auto& element : testVector)
	{
		runningSum += element.getValue();
	}

	long		secondSum = 0;

	for (size_t i = 0; i < testVector.size(); i++)
	{
		secondSum += testVector[i].getValue();
	}

	EXPECT_EQ(runningSum, secondSum);

	return(runningSum);
}


//	Run a collection of tests to insure the vector runs correctly

bool ConstructOnceResizeableVectorTest()
{
	//	Construct the vector of ten elements and insure it is correctly sized at zero initially
	
	SEFUtility::ConstructOnceResizeableVector<TestElement>		testVector(10);

	EXPECT_EQ(0, testVector.size());

	EXPECT_EQ(10, testVector.reservedSize());

	//	Resize the vector to 5 - insure it works correctly

	testVector.resize(5);

	EXPECT_TRUE( CheckAllReset( testVector ) );

	CheckSize(testVector, 5);

	long		i = 0;
	long		runningSum = 0;

	for (auto& element : testVector)
	{
		runningSum += i;
		element.setValue(i++);
	}

	EXPECT_TRUE( CheckAllNotReset( testVector ) );
	EXPECT_EQ(runningSum, ComputeRunningSum(testVector));

	//	Reset the vector, it shoud be at size zero after it is reset

	testVector.reset();

	CheckSize(testVector, 0);

	//	Reset the vector t oa size of 6

	testVector.resize(6);

	CheckSize(testVector, 6);

	EXPECT_TRUE(CheckAllReset(testVector));

	i = 0;
	runningSum = 0;

	for (auto& element : testVector)
	{
		runningSum += i;
		element.setValue(i++);
	}

	EXPECT_TRUE(CheckAllNotReset(testVector));
	EXPECT_EQ(runningSum, ComputeRunningSum(testVector));

	//	Directly resize the vector to a size of 4

	testVector.resize(4);

	CheckSize(testVector, 4);

	EXPECT_TRUE(CheckAllReset(testVector));

	//	Resize to a large vector and test

	testVector.resize(123456);

	CheckSize(testVector, 123456);

	EXPECT_TRUE(CheckAllReset(testVector));

	i = 0;
	runningSum = 0;

	for (auto& element : testVector)
	{
		runningSum += i;
		element.setValue(i++);
	}

	EXPECT_TRUE(CheckAllNotReset(testVector));
	EXPECT_EQ(runningSum, ComputeRunningSum(testVector));

	//	Finished with success

	return(true);
}


TEST(ConstructOnceResizeableVector, BasicTest)
{
  EXPECT_TRUE(ConstructOnceResizeableVectorTest());
}