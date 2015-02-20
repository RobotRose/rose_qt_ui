/*
 * dataStoreIf.h
 *
 *  Created on: Nov 18, 2010
 *      Author: martijn
 */

#ifndef DATASTOREIF_H_
#define DATASTOREIF_H_
//
#include <string>
#include <list>
#include <set>
//
#include "InfoFilesVector.h"

typedef std::vector<InformationFiles> InfoFilesVector; ///< The vector of InformationFiles
//
// ---------------------------------------------------------------------------------------------------------------------
//
class DataStoreIf
{
public:

	virtual int initialize(std::string localInfo) = 0;
	virtual int initialize(std::string defaultInfo, std::string localInfo) = 0;
    virtual int initialize(InfoFilesVector& informationFiles) = 0;

    // get primitives
    virtual bool getItemValue(std::string componentID, std::string infoID, std::string& value) = 0;
    virtual bool getItemValue(std::string componentID, std::string infoID, int& value) = 0;
    virtual bool getItemValue(std::string componentID, std::string infoID, double& value) = 0;
    virtual bool getItemValue(std::string componentID, std::string infoID, bool& value) = 0;
    // get lists primitives
    virtual bool getItemValue(std::string componentID, std::string infoID, std::list<std::string>& value) = 0;
    virtual bool getItemValue(std::string componentID, std::string infoID, std::list<int>& value) = 0;
    virtual bool getItemValue(std::string componentID, std::string infoID, std::list<double>& value) = 0;
    virtual bool getItemValue(std::string componentID, std::string infoID, std::list<bool>& value) = 0;

    // set primitives
    /// @param transactionId 0 if not part of a transaction
    virtual bool setItemValue(std::string componentID, std::string infoID, const std::string& value, int transactionId = 0) = 0;
    virtual bool setItemValue(std::string componentID, std::string infoID, const int value, int transactionId = 0) = 0;
    virtual bool setItemValue(std::string componentID, std::string infoID, const double value, int transactionId = 0) = 0;
    virtual bool setItemValue(std::string componentID, std::string infoID, const bool value, int transactionId = 0) = 0;
    // set lists primitives
    virtual bool setItemValue(std::string componentID, std::string infoID, const std::list<std::string>& value, int transactionId = 0) = 0;
    virtual bool setItemValue(std::string componentID, std::string infoID, const std::list<int>& value, int transactionId = 0) = 0;
    virtual bool setItemValue(std::string componentID, std::string infoID, const std::list<double>& value, int transactionId = 0) = 0;
    virtual bool setItemValue(std::string componentID, std::string infoID, const std::list<bool>& value, int transactionId = 0) = 0;

    // add primitives
    virtual bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::string newValue, int transactionId = 0) = 0;
    virtual bool addNewItem(std::string fileName, std::string componentID, std::string infoID, int newValue, int transactionId = 0) = 0;
    virtual bool addNewItem(std::string fileName, std::string componentID, std::string infoID, double newValue, int transactionId = 0) = 0;
    virtual bool addNewItem(std::string fileName, std::string componentID, std::string infoID, bool newValue, int transactionId = 0) = 0;
    // add lists primitives
    virtual bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<std::string>& newValue, int transactionId = 0) = 0;
    virtual bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<int>& newValue, int transactionId = 0) = 0;
    virtual bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<double>& newValue, int transactionId = 0) = 0;
    virtual bool addNewItem(std::string fileName, std::string componentID, std::string infoID, std::list<bool>& newValue, int transactionId = 0) = 0;

    virtual bool removeItem(std::string componentID, std::string infoID, int transactionId = 0) = 0;
    virtual bool removeComponent(std::string componentID, int transactionId = 0) = 0;

    virtual int startAtomicTransaction() = 0;
    virtual bool endAtomicTransaction(int id) = 0;

    virtual std::list<std::string> getRootComponents(const std::string& file) = 0;
    virtual std::set<std::string> getSubItemsFromLocal(std::string componentID, std::string itemSetID) = 0;
};
//
// ---------------------------------------------------------------------------------------------------------------------
//
#endif /* DATASTOREIF_H_ */
