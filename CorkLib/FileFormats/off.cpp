// +-------------------------------------------------------------------------
// | off.cpp
// | 
// | Author: Gilbert Bernstein
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Gilbert Bernstein 2013
// |    See the included COPYRIGHT file for further details.
// |    
// |    This file is part of the Cork library.
// |
// |    Cork is free software: you can redistribute it and/or modify
// |    it under the terms of the GNU Lesser General Public License as
// |    published by the Free Software Foundation, either version 3 of
// |    the License, or (at your option) any later version.
// |
// |    Cork is distributed in the hope that it will be useful,
// |    but WITHOUT ANY WARRANTY; without even the implied warranty of
// |    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// |    GNU Lesser General Public License for more details.
// |
// |    You should have received a copy 
// |    of the GNU Lesser General Public License
// |    along with Cork.  If not, see <http://www.gnu.org/licenses/>.
// +-------------------------------------------------------------------------
#include "files.h"


#include <boost\filesystem\fstream.hpp>
#include <boost\format.hpp>
#include <boost\iostreams\stream.hpp>
#include <boost\iostreams\device\back_inserter.hpp>
#include <boost\archive\binary_oarchive.hpp>

#include <tbb\task_group.h>



namespace Cork
{
	namespace Files
	{

		
		inline
		std::istream&		operator >> ( std::istream&								inStream,
										  Cork::Math::TriangleByIndicesBase&		triToRead )
		{
			return( inStream >> triToRead[0] >> triToRead[1] >> triToRead[2] );
		}

		inline
		std::ostream&		operator << ( std::ostream&								outStream,
										  const Cork::Math::TriangleByIndicesBase&	triToWrite )
		{
			outStream << triToWrite[0] << " " << triToWrite[1] << " " << triToWrite[2];

			return( outStream );
		}


		inline
		std::ostream& WriteVertex(std::ostream &out, const Cork::Math::Vertex3D&	vertex )
		{
			return out <<  vertex.x() << ' ' << vertex.y() << ' ' << vertex.z();
		}



		ReadFileResult		readOFF( const boost::filesystem::path&		filePath )
		{
			//	Open the mesh file

			boost::filesystem::ifstream		in( filePath );

			if ( !in.good() )
			{
				return(ReadFileResult::Failure( ReadFileResultCodes::UNABLE_TO_OPEN_FILE, boost::format( "Unable to open file: %s" ) % filePath.string() ));
			}

			//	Look for the label at the top of the file to indicate formatting of the rest of the file
			//		If the encoding is in the COFF format which includes color info, we will have to strip the color values

			std::string			fileType;

			in >> fileType;

			if ( (fileType != "OFF") && (fileType != "COFF") )
			{
				return(ReadFileResult::Failure( ReadFileResultCodes::OFF_UNRECOGNIZED_HEADER, boost::format( "Unrecognized header for OFF file: %s" ) % fileType ));
			}

			bool	stripColor = (fileType == "COFF");

			//	Load the number of vertices, faces and edges

			unsigned int numVertices, numFaces, numEdges;

			in >> numVertices >> numFaces >> numEdges;

			if ( !in.good() )
			{
				return(ReadFileResult::Failure( ReadFileResultCodes::OFF_ERROR_READING_COUNTS, "Error reading counts of vertices, faces and edges." ));
			}

			//	Get an incremental indexed vertices mesh builder

			std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder>		meshBuilder( IncrementalVertexIndexTriangleMeshBuilder::GetBuilder( numVertices, numFaces ) );

			//	Read the Vertex data

			{
				int							red, green, blue, lum;

				NUMERIC_PRECISION			x, y, z;

				for ( unsigned int i = 0; i < numVertices; ++i )
				{
					in >> x >> y >> z;

					if ( meshBuilder->AddVertex( Cork::Math::Vertex3D( x, y, z )  ) != i )
					{
						return(ReadFileResult::Failure( ReadFileResultCodes::OFF_READ_DUPLICATE_VERTICES, boost::format( "Error reading vertices - duplicate vertices found in the file: (%f,%f,%f)" ) % x % y % z ));
					}

					if ( stripColor )
					{
						in >> red >> green >> blue >> lum;
					}
				}

				if ( !in.good() )
				{
					return(ReadFileResult::Failure( ReadFileResultCodes::OFF_ERROR_READING_VERTICES, "Error reading vertices." ));
				}
			}

			// face data

			{
				TriangleMesh::TriangleByIndices		newTriangle( 0, 0, 0 );
				unsigned int						polySize;
				TriangleMeshBuilderResultCodes		resultCode;

				for ( unsigned int i = 0; i < numFaces; ++i )
				{
					in >> polySize;

					if ( polySize != 3 )
					{
						return(ReadFileResult::Failure( ReadFileResultCodes::OFF_NON_TRIANGULAR_FACE, boost::format( "Non Triangular face encountered on triangle index: %i" ) % i ));
					}

					in >> newTriangle;

					if ( (resultCode = meshBuilder->AddTriangle( newTriangle )) != TriangleMeshBuilderResultCodes::SUCCESS )
					{
						return(ReadFileResult::Failure( ReadFileResultCodes::OFF_ERROR_ADDING_FACE_TO_MESH, boost::format( "Error adding triangle to mesh encountered on triangle index: %i" ) % i ));
					}
				}

				if ( !in.good() )
				{
					return(ReadFileResult::Failure( ReadFileResultCodes::OFF_ERROR_READING_FACES, "Error reading faces." ));
				}
			}

			return(ReadFileResult( meshBuilder->Mesh() ));
		}



		WriteFileResult		writeOFF( const boost::filesystem::path&		filePath,
										const TriangleMesh&					meshToWrite )
		{
			//	Open the output file

			boost::filesystem::ofstream out( filePath );

			if ( !out.good() )
			{
				return(WriteFileResult::Failure( WriteFileResultCodes::UNABLE_TO_OPEN_FILE, (boost::format( "Unable to open file: %s" ) % filePath.c_str()).str() ));
			}

			//	Create two memory buffers so we can write the output data in two threads

			std::string																		verticesBuffer;
			verticesBuffer.reserve( 50000000 );
			boost::iostreams::back_insert_device<std::string>								verticesInserter( verticesBuffer );
			boost::iostreams::stream<boost::iostreams::back_insert_device<std::string> >	verticesStream( verticesInserter );

			std::string																		trianglesBuffer;
			trianglesBuffer.reserve( 50000000 );
			boost::iostreams::back_insert_device<std::string>								trianglesInserter( trianglesBuffer );
			boost::iostreams::stream<boost::iostreams::back_insert_device<std::string> >	trianglesStream( trianglesInserter );

			//	Set the numeric precision

			verticesStream.precision( 20 );

			//	We will be writing in 'OFF' format, no color information.  Write the format label first.

			verticesStream << "OFF\n";

			//	Write the number of vertices and triangles (i.e. faces)
			//		We will not be writing any edges, so write zero for that value.

			verticesStream << meshToWrite.numVertices() << ' ' << meshToWrite.numTriangles() << ' ' << 0 << "\n";

			//	Create a task group to allow us to spin off a thread

			tbb::task_group		taskGroup;
			
			//	Write the vertices

			taskGroup.run( [&]
			{
				for ( const auto& currentVertex : meshToWrite.vertices() )
				{
					WriteVertex( verticesStream, currentVertex ) << "\n";
				}

				verticesStream.flush();
			} );

			//	Write the triangles - they are the faces

			for ( const auto& currentTriangle : meshToWrite.triangles() )
			{
				trianglesStream << "3 " << currentTriangle << "\n";
			}

			trianglesStream.flush();

			taskGroup.wait();

			out << verticesBuffer;
			out << trianglesBuffer;

			if ( !out )
			{
				return(WriteFileResult::Failure( WriteFileResultCodes::ERROR_WRITING_TO_OFS_FILE, "Unknown Error writing to OFS file" ));
			}

			out.flush();

			return(WriteFileResult::Success());
		}



	}	//	namespace Files
}	//	namespace Cork