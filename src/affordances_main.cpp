
#include <ros/ros.h>
#include "ros_server.h"
#include <ros/serialization.h>
#include <tinyxml.h>

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

void build_simple_doc( )
{
    int list_size=10;
    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    TiXmlElement * element = new TiXmlElement( "augmented_pcl_list" );
    element->SetAttribute("size",std::to_string(list_size));
    for (int i=0;i<list_size;i++)
    {
        TiXmlElement * single_augmented_pcl = new TiXmlElement("augmented_pcl");
        TiXmlElement * pcl_xyzrgbanormal = new TiXmlElement("xyzrgbanormal");
        TiXmlText * text = new TiXmlText( "point cloud binary serialization here" );
        TiXmlElement * pcl_border = new TiXmlElement("border");
        TiXmlText * text1 = new TiXmlText( "point cloud binary serialization here" );
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
    doc.SaveFile( "madeByHand.xml" );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "affordances");
    ros::NodeHandle nh;

    ros::serialization::Serializer< pcl::PointCloud< pcl::PointXYZRGBNormal > > serializer_xyzrgbnormal;
    ros::serialization::Serializer< pcl::PointCloud< pcl::PointXYZRGBNormal > > serializer_border;
    ros::serialization::Serializer< pcl::PointCloud< pcl::PointXYZRGBNormal > > serializer_normal;

    planner::rosServer node(nh);

    node.run();

//    std_srvs::EmptyRequest req;
//    std_srvs::EmptyResponse res;
    //node.filterByCurvature(req,res); //HACK

    build_simple_doc();
    dump_to_stdout("madeByHand.xml"); // this func defined later in the tutorial

    return 0;
}
