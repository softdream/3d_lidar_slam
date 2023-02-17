#ifndef __FILE_READ_H
#define __FILE_READ_H

#include "point_cloud.h"

#include <iostream>
#include <fstream>


namespace record
{

class FileRecord
{
public:
	FileRecord()
	{

	}

	FileRecord( const std::string& file_name )
	{
		openFile( file_name );
	}

	~FileRecord()
	{

	}

	bool openFile( const std::string& file_name ) 
	{
		m_infile.open( file_name, std::ios::binary );
	
		if( !m_infile.is_open() ){
                	std::cerr<<"Failed To Open File !"<<std::endl;

                	return false;
        	}

        	std::cerr<<"Open The File !"<<std::endl;
		return true;
	}	

	void closeFile()
	{
		return m_infile.close();
	}

	template<typename T>
	bool readOneFrame( slam::PointCloud<T>& point_cloud )
	{
		m_infile.read( reinterpret_cast<char *>( &point_cloud.time_stamp ), sizeof( point_cloud.time_stamp ) );
        	m_infile.read( reinterpret_cast<char *>( &point_cloud.width ), sizeof( point_cloud.width ) );
        	m_infile.read( reinterpret_cast<char *>( &point_cloud.height ), sizeof( point_cloud.height ) );

		point_cloud.points.resize( point_cloud.width * point_cloud.height );

		m_infile.read( reinterpret_cast<char *>( point_cloud.points.data() ), point_cloud.points.size() * sizeof( T ) );
		
		return true;
	}

	const int endOfFile()
        {
                return m_infile.eof();
        }

		

private:
	std::ifstream m_infile;

};

}

#endif
