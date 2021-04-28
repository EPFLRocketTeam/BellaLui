/*
 * RemoteSensor.h
 *
 *  Created on: 7 Mar 2021
 *      Author: arion
 */

#ifndef REMOTESENSOR_H_
#define REMOTESENSOR_H_

#include "Sensor.h"

template<class T>
class RemoteSensor : public Sensor<T> {
public:
	RemoteSensor(const char* identifier) : Sensor<T>(identifier), dataPtr(nullptr) {}
	virtual ~RemoteSensor() {};
	bool load();
	bool unload();
	void onDataReception(T data);
	bool fetch(T* data);
	bool reset() { return unload() && load(); }
private:
	T storage;
	T* dataPtr;
};

template<class T>
bool RemoteSensor<T>::load() {
	dataPtr = nullptr;
	this->ready = true;
	return true;
}

template<class T>
bool RemoteSensor<T>::unload() {
	dataPtr = nullptr;
	this->ready = false;
	return true;
}

template<class T>
void RemoteSensor<T>::onDataReception(T data) {
	storage = data;
	dataPtr = &storage;
}

template<class T>
bool RemoteSensor<T>::fetch(T* data) {
	if(!this->ready)
		return false;

	if(dataPtr != nullptr) {
		memcpy(data, dataPtr, sizeof(T));
		dataPtr = nullptr;
		return true;
	} else {
		return false;
	}
}

#endif /* REMOTESENSOR_H_ */
