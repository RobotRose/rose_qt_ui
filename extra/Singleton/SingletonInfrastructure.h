/*
 * SingletonInfrastructure.h
 *
 *  Created on: Nov 23, 2010
 *      Author: martijn
 */

#ifndef SINGLETONINFRASTRUCTURE_H_
#define SINGLETONINFRASTRUCTURE_H_

/*

// --------------------------------------------------------------------
					USAGE :
// --------------------------------------------------------------------

class TestOnlyOne : public Singleton<TestOnlyOne>
{
public:
	virtual void DoIt()
	{
		std::cout << "Do it in TestOnlyOne " << std::endl;
	}
private:
	//
	friend class SingletonFactory<TestOnlyOne>;
	//
	TestOnlyOne()
	{
	}
};

SingletonFactory<TestOnlyOne>().create();

TestOnlyOne::instance()->DoIt();

// --------------------------------------------------------------------


*/

template <typename _TSingleton>
class SingletonFactory
{
public:

	SingletonFactory()
	{
	}

    virtual ~SingletonFactory()
    {

    }

    _TSingleton* create()
    {
		if (isCreated)
			return _TSingleton::mp_Instance.get();

		isCreated = true;
		_TSingleton* pTmp = new _TSingleton();
		_TSingleton::mp_Instance.reset(pTmp);

		return _TSingleton::mp_Instance.get();
    }

    template <typename _TParam>
    _TSingleton* create( _TParam param )
    {
		if (isCreated)
			return _TSingleton::mp_Instance.get();

		isCreated = true;
		_TSingleton* pTmp = new _TSingleton(param);
		_TSingleton::mp_Instance.reset(pTmp);

		return _TSingleton::mp_Instance.get();
    }

    void destroy()
    {
		_TSingleton::mp_Instance.reset();
		isCreated = false;
    }

private:

    static bool isCreated;
};

template <typename _TSingleton> bool SingletonFactory<_TSingleton>::isCreated = false;

template <typename _Ty>
class Singleton
{
public:

    static _Ty* instance()
    {
    	return mp_Instance.get();
    }

protected:

    friend class SingletonFactory<_Ty>;

    Singleton()
    {
    }

    Singleton(const Singleton& other)
    {
    }

    Singleton & operator=(const Singleton& other)
    {
    	return *this;
    }

    static std::auto_ptr<_Ty> mp_Instance;
};

template <typename _Ty> std::auto_ptr<_Ty> Singleton<_Ty>::mp_Instance;


#endif /* SINGLETONINFRASTRUCTURE_H_ */
