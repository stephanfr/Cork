/*
Copyright (c) 2013 Stephan Friedl

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Except as contained in this notice, the name(s) of the above copyright holders
shall not be used in advertising or otherwise to promote the sale, use or other
dealings in this Software without prior written authorization.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/



#ifndef RESULT_H_
#define RESULT_H_


#include <memory>
#include <string>

#include <boost/optional.hpp>
#include <boost/format.hpp>



namespace SEFUtility
{


	enum class BaseResultCodes { SUCCESS = 0, FAILURE };


	//
	//	Base Class needed primarily for passing inner errors
	//

	class ResultBase
	{
	protected :

		ResultBase( BaseResultCodes						successOrFailure,
					const std::string					message )
			: m_successOrFailure( successOrFailure ),
			  m_message( message )
		{}

		ResultBase( BaseResultCodes						successOrFailure,
					const boost::format&				message )
			: m_successOrFailure( successOrFailure ),
			  m_message( boost::str( message ) )
		{}

		ResultBase( BaseResultCodes						successOrFailure,
					const std::string					message,
					const ResultBase&					innerError )
			: m_successOrFailure( successOrFailure ),
			  m_message( message ),
			  m_innerError( innerError.shallowCopy() )
		{}

		ResultBase( BaseResultCodes						successOrFailure,
					const boost::format&				message,
					const ResultBase&					innerError )
			: m_successOrFailure( successOrFailure ),
			  m_message( boost::str( message )),
			  m_innerError( innerError.shallowCopy() )
		{}

	public :

		virtual ~ResultBase() {};


		virtual std::unique_ptr<const ResultBase>		shallowCopy() const = 0;

		bool				Succeeded() const
		{
			return( m_successOrFailure == BaseResultCodes::SUCCESS );
		}

		bool				Failed() const
		{
			return( m_successOrFailure == BaseResultCodes::FAILURE );
		}


		const std::string	message() const
		{
			return( m_message );
		}


		const std::unique_ptr<const ResultBase>&		innerError() const
		{
			return( m_innerError );
		}


		virtual const std::type_info&		errorCodeType() const = 0;
		virtual int							errorCodeValue() const = 0;


	protected :

		BaseResultCodes							m_successOrFailure;

		std::string								m_message;

		std::unique_ptr<const ResultBase>		m_innerError;
	};



	template<typename TErrorCodeEnum> class Result : public ResultBase
	{
	protected :

		typedef TErrorCodeEnum ErrorCodeType;


		Result( BaseResultCodes					successOrFailure,
				TErrorCodeEnum					errorCode,
				const std::string				message )
			: ResultBase( successOrFailure, message ),
			  m_errorCode( errorCode )
		{}

		Result( BaseResultCodes					successOrFailure,
				TErrorCodeEnum					errorCode,
				const boost::format&			message )
			: ResultBase( successOrFailure, message ),
			  m_errorCode( errorCode )
		{}

		template <typename TInnerErrorCodeEnum>
		Result( BaseResultCodes							successOrFailure,
				TErrorCodeEnum							errorCode,
				const std::string						message,
				const Result<TInnerErrorCodeEnum>&		innerError )
			: ResultBase( successOrFailure, message, innerError ),
			  m_errorCode( errorCode )
		{}

		template <typename TInnerErrorCodeEnum>
		Result( BaseResultCodes							successOrFailure,
				TErrorCodeEnum							errorCode,
				const boost::format&					message,
				const Result<TInnerErrorCodeEnum>&		innerError )
			: ResultBase( successOrFailure, message, innerError ),
			  m_errorCode( errorCode )
		{}


		std::unique_ptr<const ResultBase>		shallowCopy() const
		{
			return( std::unique_ptr<const ResultBase>( new Result<TErrorCodeEnum>( *this )));
		}


	public :

		Result( const Result<TErrorCodeEnum>&	resultToCopy )
			: ResultBase( resultToCopy.m_successOrFailure, resultToCopy.m_message ),
			  m_errorCode( resultToCopy.m_errorCode )
		{
			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );
		}

		virtual ~Result() {};



		const Result<TErrorCodeEnum>&		operator=( const Result<TErrorCodeEnum>&	resultToCopy )
		{
			m_successOrFailure = resultToCopy.m_successOrFailure;
			m_message = resultToCopy.m_message;
			m_errorCode = resultToCopy.m_errorCode;

			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );

			return( *this );
		}


		static Result<TErrorCodeEnum>		Success()
		{
			return( Result( BaseResultCodes::SUCCESS, TErrorCodeEnum::SUCCESS, "Success" ));
		};

		static Result<TErrorCodeEnum>		Failure( TErrorCodeEnum					errorCode,
													 const std::string&				message )
		{
			return( Result( BaseResultCodes::FAILURE, errorCode, message ));
		}

		static Result<TErrorCodeEnum>		Failure( TErrorCodeEnum					errorCode,
													 const boost::format&			message )
		{
			return( Result( BaseResultCodes::FAILURE, errorCode, message ) );
		}

		template <typename TInnerErrorCodeEnum>
		static Result<TErrorCodeEnum>		Failure( TErrorCodeEnum							errorCode,
													 const std::string&						message,
													 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( Result( BaseResultCodes::FAILURE, errorCode, message, innerError ));
		}

		template <typename TInnerErrorCodeEnum>
		static Result<TErrorCodeEnum>		Failure( TErrorCodeEnum							errorCode,
													 const boost::format&					message,
													 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( Result( BaseResultCodes::FAILURE, errorCode, message, innerError ) );
		}


		TErrorCodeEnum				errorCode() const
		{
			return( m_errorCode );
		}

		const std::type_info&		errorCodeType() const
		{
			return( typeid( ErrorCodeType ) );
		}

		int							errorCodeValue() const
		{
			return( (int)m_errorCode );
		}


	protected :

		TErrorCodeEnum				m_errorCode;
	};



	template <typename TErrorCodeEnum, typename TResultType> class ResultWithReturnValue : public Result<TErrorCodeEnum>
	{
	protected :

		ResultWithReturnValue( BaseResultCodes					successOrFailure,
							   TErrorCodeEnum					errorCode,
							   const std::string				message,
							   TResultType						returnValue )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message ),
			  m_returnValue( returnValue )
		{}

		ResultWithReturnValue( BaseResultCodes					successOrFailure,
							   TErrorCodeEnum					errorCode,
							   const boost::format&				message,
							   TResultType						returnValue )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message ),
			  m_returnValue( returnValue )
		{}

		ResultWithReturnValue( BaseResultCodes					successOrFailure,
							   TErrorCodeEnum					errorCode,
							   const std::string				message )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message )
		{}

		ResultWithReturnValue( BaseResultCodes					successOrFailure,
							   TErrorCodeEnum					errorCode,
							   const boost::format&				message )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message )
		{}

		template <typename TInnerErrorCodeEnum>
		ResultWithReturnValue( BaseResultCodes							successOrFailure,
							   TErrorCodeEnum							errorCode,
							   const std::string						message,
							   const Result<TInnerErrorCodeEnum>&		innerError )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message, innerError )
		{}

		template <typename TInnerErrorCodeEnum>
		ResultWithReturnValue( BaseResultCodes							successOrFailure,
							   TErrorCodeEnum							errorCode,
							   const boost::format&						message,
							   const Result<TInnerErrorCodeEnum>&		innerError )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message, innerError )
		{}

	public :

		ResultWithReturnValue( TResultType						returnValue )
			: Result<TErrorCodeEnum>( BaseResultCodes::SUCCESS, TErrorCodeEnum::SUCCESS, "Success" ),
			  m_returnValue( returnValue )
		{}

		ResultWithReturnValue( const ResultWithReturnValue&		resultToCopy )
			: Result<TErrorCodeEnum>( resultToCopy.m_successOrFailure, resultToCopy.m_errorCode, resultToCopy.m_message ),
			  m_returnValue( resultToCopy.m_returnValue )
		{
			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );
		}


		virtual ~ResultWithReturnValue() {};




		operator	const Result<TErrorCodeEnum>&() const
		{
			return( Result<TErrorCodeEnum>( this->m_successOrFailure, this->m_errorCode, this->m_message ) );
		}

		const ResultWithReturnValue<TErrorCodeEnum, TResultType>&		operator=( const ResultWithReturnValue<TErrorCodeEnum, TResultType>&	resultToCopy )
		{
			ResultBase::m_successOrFailure = resultToCopy.m_successOrFailure;
			ResultBase::m_message = resultToCopy.m_message;
			Result<TErrorCodeEnum>::m_errorCode = resultToCopy.m_errorCode;
			m_returnValue = resultToCopy.m_returnValue;

			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );

			return( *this );
		}



		static ResultWithReturnValue<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum				errorCode,
									 	 	 	 	 	 	 	 	 	 	 	 const std::string&			message )
		{
			return( ResultWithReturnValue( BaseResultCodes::FAILURE, errorCode, message ));
		}

		static ResultWithReturnValue<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum				errorCode,
									 	 	 	 	 	 	 	 	 	 	 	 const boost::format&		message )
		{
			return( ResultWithReturnValue( BaseResultCodes::FAILURE, errorCode, message ));
		}

		template <typename TInnerErrorCodeEnum>
		static ResultWithReturnValue<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
																				 const std::string&						message,
																				 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( ResultWithReturnValue( BaseResultCodes::FAILURE, errorCode, message, innerError ));
		}

		template <typename TInnerErrorCodeEnum>
		static ResultWithReturnValue<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
																				 const boost::format&					message,
																				 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( ResultWithReturnValue( BaseResultCodes::FAILURE, errorCode, message, innerError ));
		}


		TResultType&			ReturnValue()
		{
			assert( m_returnValue.is_initialized() );
			return( *m_returnValue );
		}

	protected :

		boost::optional<TResultType>			m_returnValue;
	};




	template <typename TErrorCodeEnum, typename TResultType> class ResultWithReturnRef : public Result<TErrorCodeEnum>
	{
	protected :

		ResultWithReturnRef( BaseResultCodes						successOrFailure,
							 TErrorCodeEnum							errorCode,
							 const std::string						message,
							 TResultType&							returnRef )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message ),
			  m_returnRef( returnRef )
		{}


		ResultWithReturnRef( BaseResultCodes						successOrFailure,
							 TErrorCodeEnum							errorCode,
							 const boost::format&					message,
							 TResultType&							returnRef )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message ),
			  m_returnRef( returnRef )
		{}

		ResultWithReturnRef( BaseResultCodes						successOrFailure,
							 TErrorCodeEnum							errorCode,
							 const std::string						message )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message )
		{}

		ResultWithReturnRef( BaseResultCodes						successOrFailure,
							 TErrorCodeEnum							errorCode,
							 const boost::format&					message )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message )
		{}

		template <typename TInnerErrorCodeEnum>
		ResultWithReturnRef( BaseResultCodes						successOrFailure,
							 TErrorCodeEnum							errorCode,
							 const std::string						message,
							 const Result<TInnerErrorCodeEnum>&		innerError )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message, innerError )
		{}

		template <typename TInnerErrorCodeEnum>
		ResultWithReturnRef( BaseResultCodes						successOrFailure,
							 TErrorCodeEnum							errorCode,
							 const boost::format&					message,
							 const Result<TInnerErrorCodeEnum>&		innerError )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message, innerError )
		{}

	public :

		ResultWithReturnRef( TResultType&					returnRef )
			: Result<TErrorCodeEnum>( BaseResultCodes::SUCCESS, TErrorCodeEnum::SUCCESS, "Success" ),
			  m_returnRef( returnRef )
		{}

		ResultWithReturnRef( const ResultWithReturnRef&		resultToCopy )
			: Result<TErrorCodeEnum>( resultToCopy.m_successOrFailure, resultToCopy.m_errorCode, resultToCopy.m_message ),
			  m_returnRef( resultToCopy.m_returnRef )
		{
			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );
		}


		virtual ~ResultWithReturnRef() {};


		operator	const Result<TErrorCodeEnum>&() const
		{
			return( Result<TErrorCodeEnum>( this->m_successOrFailure, this->m_errorCode, this->m_message ) );
		}


		const ResultWithReturnRef<TErrorCodeEnum, TResultType>&		operator=( const ResultWithReturnRef<TErrorCodeEnum, TResultType>&	resultToCopy )
		{
			ResultBase::m_successOrFailure = resultToCopy.m_successOrFailure;
			ResultBase::m_message = resultToCopy.m_message;
			Result<TErrorCodeEnum>::m_errorCode = resultToCopy.m_errorCode;
			m_returnRef = resultToCopy.m_returnRef;

			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );

			return( *this );
		}


		static ResultWithReturnRef<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
									 	 	 	 	 	 	 	 	 	 	 const std::string&						message )
		{
			return( ResultWithReturnRef( BaseResultCodes::FAILURE, errorCode, message ));
		}

		static ResultWithReturnRef<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
									 	 	 	 	 	 	 	 	 	 	 const boost::format&					message )
		{
			return( ResultWithReturnRef( BaseResultCodes::FAILURE, errorCode, message ));
		}

		template <typename TInnerErrorCodeEnum>
		static ResultWithReturnRef<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
																			 const std::string&						message,
																			 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( ResultWithReturnRef( BaseResultCodes::FAILURE, errorCode, message, innerError ));
		}

		template <typename TInnerErrorCodeEnum>
		static ResultWithReturnRef<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
																			 const boost::format&					message,
																			 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( ResultWithReturnRef( BaseResultCodes::FAILURE, errorCode, message, innerError ));
		}


		TResultType&			ReturnRef()
		{
			assert( m_returnRef.is_initialized() );

			return( *m_returnRef );
		}

	protected :

		boost::optional<TResultType&>			m_returnRef;
	};




	template <typename TErrorCodeEnum, typename TResultType> class ResultWithUniqueReturnPtr : public Result<TErrorCodeEnum>
	{
	private :

		ResultWithUniqueReturnPtr( BaseResultCodes						successOrFailure,
							 	   TErrorCodeEnum						errorCode,
							 	   const std::string					message )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message )
		{}

		ResultWithUniqueReturnPtr( BaseResultCodes						successOrFailure,
							 	   TErrorCodeEnum						errorCode,
							 	   const boost::format&					message )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message )
		{}

		template <typename TInnerErrorCodeEnum>
		ResultWithUniqueReturnPtr( BaseResultCodes							successOrFailure,
								   TErrorCodeEnum							errorCode,
								   const std::string						message,
								   const Result<TInnerErrorCodeEnum>&		innerError )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message, innerError )
		{}

		template <typename TInnerErrorCodeEnum>
		ResultWithUniqueReturnPtr( BaseResultCodes							successOrFailure,
								   TErrorCodeEnum							errorCode,
								   const boost::format&						message,
								   const Result<TInnerErrorCodeEnum>&		innerError )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message, innerError )
		{}

	public :

		explicit
		ResultWithUniqueReturnPtr( std::unique_ptr<TResultType>&			returnValue )
			: Result<TErrorCodeEnum>( BaseResultCodes::SUCCESS, TErrorCodeEnum::SUCCESS, "Success" ),
			  m_returnPtr( std::move( returnValue ))
			  {}

//		explicit
//		ResultWithUniqueReturnPtr( std::unique_ptr<TResultType>			returnValue )
//			: Result<TErrorCodeEnum>( BaseResultCodes::SUCCESS, TErrorCodeEnum::SUCCESS, "Success" ),
//			  m_returnPtr( std::move( returnValue ))
//			  {}

		ResultWithUniqueReturnPtr( ResultWithUniqueReturnPtr<TErrorCodeEnum,TResultType>&		resultToCopy )
			: Result<TErrorCodeEnum>( resultToCopy.m_successOrFailure, resultToCopy.m_errorCode, resultToCopy.m_message ),
			  m_returnPtr( std::move( resultToCopy.m_returnPtr ))
		{
			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );
		}

		virtual ~ResultWithUniqueReturnPtr() {};



		operator	const Result<TErrorCodeEnum>&() const
		{
			return( Result<TErrorCodeEnum>( this->m_successOrFailure, this->m_errorCode, this->m_message ) );
		}


		const ResultWithUniqueReturnPtr<TErrorCodeEnum, TResultType>&		operator=( const ResultWithUniqueReturnPtr<TErrorCodeEnum, TResultType>&	resultToCopy )
		{
			ResultBase::m_successOrFailure = resultToCopy.m_successOrFailure;
			ResultBase::m_message = resultToCopy.m_message;
			Result<TErrorCodeEnum>::m_errorCode = resultToCopy.m_errorCode;
			m_returnPtr = resultToCopy.m_returnPtr;

			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );

			return( *this );
		}



		static ResultWithUniqueReturnPtr<TErrorCodeEnum, TResultType>		Success( TResultType*	returnValue )
		{
			return(ResultWithUniqueReturnPtr( std::unique_ptr<TResultType>( returnValue ) ));
		}

		static ResultWithUniqueReturnPtr<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum			errorCode,
									 	 	 	 	 	 	 	 	 	 	 	 	 const std::string&		message )
		{
			return( ResultWithUniqueReturnPtr( BaseResultCodes::FAILURE, errorCode, message ));
		}

		static ResultWithUniqueReturnPtr<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum			errorCode,
									 	 	 	 	 	 	 	 	 	 	 	 	 const boost::format&	message )
		{
			return( ResultWithUniqueReturnPtr( BaseResultCodes::FAILURE, errorCode, message ));
		}

		template <typename TInnerErrorCodeEnum>
		static ResultWithUniqueReturnPtr<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
																					 const std::string&						message,
																					 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( ResultWithUniqueReturnPtr( BaseResultCodes::FAILURE, errorCode, message, innerError ));
		}

		template <typename TInnerErrorCodeEnum>
		static ResultWithUniqueReturnPtr<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
																					 const boost::format&					message,
																					 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( ResultWithUniqueReturnPtr( BaseResultCodes::FAILURE, errorCode, message, innerError ));
		}


		std::unique_ptr<TResultType>&			ReturnPtr()
		{
			return( m_returnPtr );
		}

	private :

		std::unique_ptr<TResultType>			m_returnPtr;
	};





	template <typename TErrorCodeEnum, typename TResultType> class ResultWithSharedReturnPtr : public Result<TErrorCodeEnum>
	{
	private :

		ResultWithSharedReturnPtr( BaseResultCodes						successOrFailure,
							 	   TErrorCodeEnum						errorCode,
							 	   const std::string					message )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message )
		{}

		ResultWithSharedReturnPtr( BaseResultCodes						successOrFailure,
							 	   TErrorCodeEnum						errorCode,
							 	   const boost::format&					message )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message )
		{}

		template <typename TInnerErrorCodeEnum>
		ResultWithSharedReturnPtr( BaseResultCodes						successOrFailure,
								   TErrorCodeEnum							errorCode,
								   const std::string						message,
								   const Result<TInnerErrorCodeEnum>&		innerError )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message, innerError )
		{}

		template <typename TInnerErrorCodeEnum>
		ResultWithSharedReturnPtr( BaseResultCodes						successOrFailure,
								   TErrorCodeEnum							errorCode,
								   const boost::format&						message,
								   const Result<TInnerErrorCodeEnum>&		innerError )
			: Result<TErrorCodeEnum>( successOrFailure, errorCode, message, innerError )
		{}

	public :

		ResultWithSharedReturnPtr( std::shared_ptr<TResultType>&			returnValue )
			: Result<TErrorCodeEnum>( BaseResultCodes::SUCCESS, TErrorCodeEnum::SUCCESS, "Success" ),
			  m_returnPtr( std::move( returnValue ))
			  {}

		ResultWithSharedReturnPtr( std::shared_ptr<TResultType>			returnValue )
			: Result<TErrorCodeEnum>( BaseResultCodes::SUCCESS, TErrorCodeEnum::SUCCESS, "Success" ),
			  m_returnPtr( std::move( returnValue ))
			  {}

		ResultWithSharedReturnPtr( const ResultWithSharedReturnPtr<TErrorCodeEnum,TResultType>&		resultToCopy )
			: Result<TErrorCodeEnum>( resultToCopy.m_successOrFailure, resultToCopy.m_errorCode, resultToCopy.m_message )
		{
			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );
		}

		virtual ~ResultWithSharedReturnPtr() {};



		operator	const Result<TErrorCodeEnum>&() const
		{
			return( Result<TErrorCodeEnum>( this->m_successOrFailure, this->m_errorCode, this->m_message ) );
		}


		const ResultWithSharedReturnPtr<TErrorCodeEnum, TResultType>&		operator=( const ResultWithSharedReturnPtr<TErrorCodeEnum, TResultType>&	resultToCopy )
		{
			ResultBase::m_successOrFailure = resultToCopy.m_successOrFailure;
			ResultBase::m_message = resultToCopy.m_message;
			Result<TErrorCodeEnum>::m_errorCode = resultToCopy.m_errorCode;
			m_returnPtr = resultToCopy.m_returnPtr;

			m_innerError = ( resultToCopy.m_innerError ? resultToCopy.m_innerError->shallowCopy() : nullptr );

			return( *this );
		}



		static ResultWithSharedReturnPtr<TErrorCodeEnum,TResultType>		Success( std::shared_ptr<TResultType>&	returnValue ) = delete;

		static ResultWithSharedReturnPtr<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum			errorCode,
									 	 	 	 	 	 	 	 	 	 	 	 	 const std::string&		message )
		{
			return( ResultWithSharedReturnPtr( BaseResultCodes::FAILURE, errorCode, message ));
		}

		static ResultWithSharedReturnPtr<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum				errorCode,
									 	 	 	 	 	 	 	 	 	 	 	 	 const boost::format&		message )
		{
			return( ResultWithSharedReturnPtr( BaseResultCodes::FAILURE, errorCode, message ));
		}

		template <typename TInnerErrorCodeEnum>
		static ResultWithSharedReturnPtr<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
																					 const std::string&						message,
																					 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( ResultWithSharedReturnPtr( BaseResultCodes::FAILURE, errorCode, message, innerError ));
		}

		template <typename TInnerErrorCodeEnum>
		static ResultWithSharedReturnPtr<TErrorCodeEnum,TResultType>		Failure( TErrorCodeEnum							errorCode,
																					 const boost::format&					message,
																					 const Result<TInnerErrorCodeEnum>&		innerError )
		{
			return( ResultWithSharedReturnPtr( BaseResultCodes::FAILURE, errorCode, message, innerError ));
		}


		std::shared_ptr<TResultType>&			ReturnPtr()
		{
			return( m_returnPtr );
		}

	private :

		std::shared_ptr<TResultType>			m_returnPtr;
	};



} /* namespace Utility */
#endif /* RESULT_H_ */
