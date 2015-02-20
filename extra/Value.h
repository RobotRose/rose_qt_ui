#ifndef VALUE_H_
#define VALUE_H_


// Value.h
// Copyright Sioux 2010
// Initial Creation date: Sep 2, 2010
//
// Description: Header file for Value.cpp

#include <string>
#include <vector>

class Value
{
public:
    Value() {}
    Value(int value) : m_value_int(value) {}
    Value(const std::string& value) : m_value_string(value) {}
    Value(const unsigned char * value) : m_value_string(reinterpret_cast<const char*>(value)) {}
    Value(const unsigned char * value, int number_of_elements)
        : m_value_vector(value,&value[number_of_elements]) {}
    Value(const std::vector<unsigned char>& vec) : m_value_vector(vec) {}
    ~Value() {}
    operator int() const { return m_value_int; }
    operator std::string() const { return m_value_string; }
    operator std::vector<unsigned char>() const { return m_value_vector; }
private:
    int m_value_int ;
    std::string m_value_string ;
    std::vector<unsigned char> m_value_vector ;
};

#endif /* VALUE_H_ */
