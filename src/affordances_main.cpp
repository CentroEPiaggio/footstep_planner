/* Copyright [2014] [Mirko Ferrati, Alessandro Settimi, Corrado Pavan, Carlos J Rosales]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/


#include <ros/ros.h>
#include "ros_server.h"
#include <tinyxml.h>
#include <ostream>
#include <sstream>
#include <data_types.h>
#include <xml_pcl_io.h>
// ----------------------------------------------------------------------
// STDOUT dump and indenting utility functions
// ----------------------------------------------------------------------
const unsigned int NUM_INDENTS_PER_SPACE=2;

const char * getIndent( unsigned int numIndents )
{
    static const char * pINDENT="                                      + ";
    static const unsigned int LENGTH=strlen( pINDENT );
    unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
    if ( n > LENGTH ) n = LENGTH;

    return &pINDENT[ LENGTH-n ];
}

// same as getIndent but no "+" at the end
const char * getIndentAlt( unsigned int numIndents )
{
    static const char * pINDENT="                                        ";
    static const unsigned int LENGTH=strlen( pINDENT );
    unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
    if ( n > LENGTH ) n = LENGTH;

    return &pINDENT[ LENGTH-n ];
}

int dump_attribs_to_stdout(TiXmlElement* pElement, unsigned int indent)
{
    if ( !pElement ) return 0;

    TiXmlAttribute* pAttrib=pElement->FirstAttribute();
    int i=0;
    int ival;
    double dval;
    const char* pIndent=getIndent(indent);
    printf("\n");
    while (pAttrib)
    {
        printf( "%s%s: value=[%s]", pIndent, pAttrib->Name(), pAttrib->Value());

        if (pAttrib->QueryIntValue(&ival)==TIXML_SUCCESS)    printf( " int=%d", ival);
        if (pAttrib->QueryDoubleValue(&dval)==TIXML_SUCCESS) printf( " d=%1.1f", dval);
        printf( "\n" );
        i++;
        pAttrib=pAttrib->Next();
    }
    return i;
}

void dump_to_stdout( TiXmlNode* pParent, unsigned int indent = 0 )
{
    if ( !pParent ) return;

    TiXmlNode* pChild;
    TiXmlText* pText;
    int t = pParent->Type();
    printf( "%s", getIndent(indent));
    int num;

    switch ( t )
    {
    case TiXmlNode::TINYXML_DOCUMENT:
        printf( "Document" );
        break;

    case TiXmlNode::TINYXML_ELEMENT:
        printf( "Element [%s]", pParent->Value() );
        num=dump_attribs_to_stdout(pParent->ToElement(), indent+1);
        switch(num)
        {
            case 0:  printf( " (No attributes)"); break;
            case 1:  printf( "%s1 attribute", getIndentAlt(indent)); break;
            default: printf( "%s%d attributes", getIndentAlt(indent), num); break;
        }
        break;

    case TiXmlNode::TINYXML_COMMENT:
        printf( "Comment: [%s]", pParent->Value());
        break;

    case TiXmlNode::TINYXML_UNKNOWN:
        printf( "Unknown" );
        break;

    case TiXmlNode::TINYXML_TEXT:
        pText = pParent->ToText();
        printf( "Text: [%s]", pText->Value() );
        break;

    case TiXmlNode::TINYXML_DECLARATION:
        printf( "Declaration" );
        break;
    default:
        break;
    }
    printf( "\n" );
    for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
        dump_to_stdout( pChild, indent+1 );
    }
}

// load the named file and dump its structure to STDOUT
void dump_to_stdout(const char* pFilename)
{
    TiXmlDocument doc(pFilename);
    bool loadOkay = doc.LoadFile();
    if (loadOkay)
    {
        printf("\n%s:\n", pFilename);
        dump_to_stdout( &doc ); // defined later in the tutorial
    }
    else
    {
        printf("Failed to load file \"%s\"\n", pFilename);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "affordances");
    ros::NodeHandle nh;
    xml_pcl_io file_manager;

    planner::rosServer node(nh);

    node.run();

    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    node.filterByCurvature(req,res); //HACK
    std::string filename="madeByHand.xml";
    file_manager.write_to_file(filename,node.polygons);
    std::ifstream file;
    file.open(filename);
    assert(file.is_open());
    //dump_to_stdout(filename.c_str());
    node.polygons.clear();
    file_manager.read_from_file(filename, node.polygons);

    return 0;
}
