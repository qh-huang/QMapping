#ifndef LOGUTIL_H
#define LOGUTIL_H

#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

/* qiao@2015.09.21: CSV utils, refer to http://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c */
class LogRow
{
public:
	LogRow(char token = ',') { m_token = token; }
	std::string const& operator[](std::size_t index) const { return m_data[index]; }
	std::size_t size() const { return m_data.size(); }
	inline void readNextRow(std::istream& str) 
	{
		std::string line;
		std::getline(str, line);

		std::stringstream   lineStream(line);
		std::string         cell;

		m_data.clear();
		while (std::getline(lineStream, cell, m_token))
		{
			m_data.push_back(cell);
		}
	}
private:
	std::vector<std::string>    m_data;
	char m_token;
};

std::istream& operator>>(std::istream& str, LogRow& data)
{
	data.readNextRow(str);
	return str;
}

class LogIterator
{
public:
	typedef std::input_iterator_tag     iterator_category;
	typedef LogRow                      value_type;
	typedef std::size_t                 difference_type;
	typedef LogRow*                     pointer;
	typedef LogRow&                     reference;

	LogIterator(std::istream& str, char token = ' ') :m_str(str.good() ? &str : NULL) , m_row(LogRow(token)) { ++(*this); }
	LogIterator() :m_str(NULL) {}

	// Pre Increment
	LogIterator& operator++()               { if (m_str) { (*m_str) >> m_row; m_str = m_str->good() ? m_str : NULL; } return *this; }
	// Post increment
	LogIterator operator++(int)             { LogIterator    tmp(*this); ++(*this); return tmp; }
	LogRow const& operator*()   const       { return m_row; }
	LogRow const* operator->()  const       { return &m_row; }

	bool operator==(LogIterator const& rhs) { return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL))); }
	bool operator!=(LogIterator const& rhs) { return !((*this) == rhs); }
private:
	std::istream*       m_str;
	LogRow              m_row;
};

#endif // LOGUTIL_H