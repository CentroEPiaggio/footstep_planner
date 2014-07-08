#include "xml_pcl_io.h"
#include <tinyxml.h>
#include <ostream>
#include <sstream>
#include <pcd_stream_io.hpp>
xml_pcl_io::xml_pcl_io()
{
}


bool xml_pcl_io::write_to_file(std::string filename, const std::list<planner::polygon_with_normals>& clusters )
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
    return doc.SaveFile( filename );

}

bool xml_pcl_io::read_from_file(std::string filename, std::list<planner::polygon_with_normals>& clusters )
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
