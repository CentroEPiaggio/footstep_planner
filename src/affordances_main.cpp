
#include <ros/ros.h>
#include "ros_server.h"
#include <tinyxml.h>
#include <ostream>
#include <sstream>
#include <pcd_stream_io.hpp>
#include <curvaturefilter.h>
#include <borderextraction.h>
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

void build_simple_doc(std::string filename, const std::list<planner::polygon_with_normals>& clusters )
{
    pcl::StreamWriter writer;

    int list_size=clusters.size();
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    TiXmlElement * element = new TiXmlElement( "augmented_pcl_list" );
    element->SetAttribute("size",std::to_string(list_size));
    for (auto& cluster:clusters)
    {
        TiXmlElement * single_augmented_pcl = new TiXmlElement("augmented_pcl");
        TiXmlElement * pcl_xyzrgbanormal = new TiXmlElement("xyzrgbanormal");
        std::ostringstream temp;
        writer.writeASCII(temp,*(cluster.normals));
        TiXmlText * text = new TiXmlText( temp.str() );

        TiXmlElement * pcl_border = new TiXmlElement("border");
        std::ostringstream temp1;
        writer.writeASCII(temp1,*(cluster.border));
        TiXmlText * text1 = new TiXmlText( temp1.str() );

        TiXmlElement * pcl_normal = new TiXmlElement("normal");
        TiXmlText * text2 = new TiXmlText( "point cloud binary serialization here" );

        element->LinkEndChild(single_augmented_pcl);
        single_augmented_pcl->LinkEndChild(pcl_xyzrgbanormal);
        single_augmented_pcl->LinkEndChild(pcl_border);
        single_augmented_pcl->LinkEndChild(pcl_normal);
        pcl_xyzrgbanormal->LinkEndChild(text);
        pcl_border->LinkEndChild(text1);
        pcl_normal->LinkEndChild(text2);
    }
    doc.LinkEndChild( decl );
    doc.LinkEndChild( element );
    doc.SaveFile( filename );
}

bool read_simple_doc(std::string filename, std::list<planner::polygon_with_normals>& clusters )
{
    TiXmlDocument doc(filename);
    bool loadOkay = doc.LoadFile();
    if (!loadOkay)
    {
        printf("Failed to load file \"%s\"\n", filename.c_str());
        return false;
    }

    pcl::StreamReader reader;
    int t = doc.Type();
    if (t!=TiXmlNode::TINYXML_DOCUMENT)
    {
        printf("Expected a document at the beginning of the file %s, found something else \n",filename.c_str());
        return false;
    }


    TiXmlNode* pcl_array=doc.FirstChild("augmented_pcl_list");
    if (pcl_array==NULL || pcl_array->Type()!=TiXmlNode::TINYXML_ELEMENT)
    {
        printf("Could not find element %s into file %s\n","augmented_pcl_list",filename.c_str());
        return false;
    }

    TiXmlNode* augmented_pcl;
    for ( augmented_pcl = pcl_array->FirstChild(); augmented_pcl != 0; augmented_pcl = augmented_pcl->NextSibling())
    {
        t=augmented_pcl->Type();
        if (t==TiXmlNode::TINYXML_ELEMENT && augmented_pcl->ValueStr()=="augmented_pcl")
        {
            planner::polygon_with_normals polygon;
            TiXmlNode* single_pcl=augmented_pcl->FirstChild("xyzrgbanormal");
            if (single_pcl==NULL)
                printf("found an %s element without a %s child, xml is malformed \n","augmented_pcl_list","augmented_pcl");
            std::string pointcloud=single_pcl->FirstChild()->ToText()->Value();
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            std::istringstream input(pointcloud);
            reader.read(input,*temp);
            polygon.normals=temp;

            single_pcl=augmented_pcl->FirstChild("border");
            if (single_pcl==NULL)
                printf("found an %s element without a %s child, xml is malformed \n","augmented_pcl_list","border");
            pointcloud=single_pcl->FirstChild()->ToText()->Value();
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZ>);
            std::istringstream input1(pointcloud);
            reader.read(input1,*temp1);
            polygon.border=temp1;
            clusters.push_back(polygon);
        }
        else
            printf("child of %s ignored since it is not an element, is the xml version different?\n","augmented_pcl_list");
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "affordances");
    ros::NodeHandle nh;


    planner::rosServer node(nh);

    node.run();

    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    node.filterByCurvature(req,res); //HACK
    std::string filename="madeByHand.xml";
    build_simple_doc(filename,node.polygons);
    std::ifstream file;
    file.open(filename);
    assert(file.is_open());
    //dump_to_stdout(filename.c_str());
    node.polygons.clear();
    read_simple_doc(filename, node.polygons);

    return 0;
}
