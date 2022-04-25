/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Helper class for safe intefrace storage
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _TREF_H_
#define _TREF_H_

#include <stddef.h>
#include <assert.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/Log.h>

namespace artec { namespace sdk { namespace base
{

/**
 *  This class is intended for safe manipulation of supported classes in terms of their
 *  time frame. Those classes should implement IRef interface that provides reference 
 *  counting support.
 *  TRef objects should hold pointers to the instances of such classes.
 *  Below is an example of how to create an instance of an IFrameMesh 
 *  implementation:
 *      TRef<IFrameMesh> mesh;
 *      createFrameMesh( &mesh );
 *  Operator &() is overloaded so that the createFrameMesh() function can fill the passed TRef
 *  with a newly created instance. At the very moment of definition TRef object has a count
 *  of references equal to 1.
 *
 *  TRef instance represents a smart pointer paradigm, so the ->() operator is also overloaded
 *  in order for the object to be used as a normal pointer: 
 *      bool isTextured = mesh->isTextured();
 *
 *  TRef can also be used as a deletion controller:
 *  {
 *      TRef<IFrameMesh> mesh;
 *      createFrameMesh( &mesh );
 *      bool isTextured = mesh->isTextured();
 *  } 
 *  At this point, the TRef object is to be deleted. This will causes deletion of the FrameMesh instance,
 *  once the number of references counts downs to 0.
 *  
 *  Note that all objects in the API should be created using the appropriate createXXX() 
 *  functions. So, in order to use this technique, one should create an IRef implementation
 *  and the createXXX() function as it is not permitted to use clauses like the following one:
 *      TRef<IMyObject> mine( new MyObject );
 *  This leads to memory leak as the count of references will equal 2 (1 from the object 
 *  constructor and 1 from the TRef constructor. If you need to initialize TRef directly,
 *  do it as follows:
 *      TRef<IMyObject> object;
 *      object.attach( new MyObject() );
 */      

template <class T> class TRef
{
public:
	typedef T base;

	TRef(T* p = NULL)
	{
		tObject_ = p;
		if (p)
			p->addRef();
	}

	TRef(const TRef<T>& p)
	{
		tObject_ = p.tObject_;

		if (tObject_)
			tObject_->addRef();
	}

	TRef(TRef<T>&& p)
	{
		tObject_ = p.tObject_;
		p.tObject_ = NULL;
	}

	~TRef()
	{
		if (tObject_)
			tObject_->release();
	}

	operator bool() const { return tObject_ != NULL; }

	T* operator ->() const { assert( tObject_ != NULL ); return tObject_; }

	operator T*() const { return tObject_; }

	const T& operator *() const { assert( tObject_ != NULL ); return *tObject_; }

	T& operator *() { assert( tObject_ != NULL ); return *tObject_; }

	TRef<T>& operator=(T* p)
	{
		T* tmpObject = tObject_;

		tObject_ = p;

		if (tObject_)
			tObject_->addRef();

		if (tmpObject)
			tmpObject->release();

		return *this;
	}

	TRef<T>& operator=(const TRef<T>& p)
	{
		T* tmpObject = tObject_;

		tObject_ = p;

		if (tObject_)
			tObject_->addRef();

		if (tmpObject)
			tmpObject->release();

		return *this;
	}

	/// Move operator
	TRef<T>& operator=(TRef<T>&& p)
	{
		// Disable operator&() overloading
		void* addressP = reinterpret_cast<void*>(&const_cast<char&>(reinterpret_cast<const volatile char&>(p)));

		if (this != addressP)
		{
			if (tObject_)
				tObject_->release();

			tObject_ = p.tObject_;
			p.tObject_ = NULL;
		}
		return *this;
	}

	T** operator&()
	{
        if (tObject_ != NULL)
        {
            ABASESDK_ERROR(L"Please release your TRef before using operator&");
            return NULL;
        }

		return &tObject_;
	}

	bool operator==(T* p) const
	{
		return tObject_ == p;
	}

	bool operator!=(T* p) const
	{
		return !(tObject_ == p);
	}

	/// Attach to the interface pointer without calling addref() for it
	void attach(T* p)
	{
		if (tObject_)
			tObject_->release();

		tObject_ = p;
	}

	/// Detach the interface pointer without calling release() for it
	T* detach()
	{
		T* tmpObject = tObject_;
		tObject_ = NULL;
		return tmpObject;
	}

	void release()
	{
		if (tObject_ == NULL)
			return;

		tObject_->release();
		tObject_ = NULL;
	}

protected:
	T* tObject_;

};

} } } // namespace artec::sdk::base

#endif // _TREF_H_
