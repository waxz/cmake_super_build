//
// Created by waxz on 22-10-10.
//
#include <string>
#include <iostream>
#include "tinyxml2.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace tinyxml2;

std::string xml2json(std::string &src)
{
    XMLDocument doc;
    doc.Parse( src.c_str() );

    json  root;
    XMLElement* rootElement = doc.RootElement();
    XMLElement* child = rootElement->FirstChildElement();
    while(child) {
        const char* title = child->Name() ;
        const char* value = child->GetText();
        child = child->NextSiblingElement();
        root[title]=value ;
    }
    return  root.dump() ;
}

std::string json2xml(std::string& src)
{
    XMLDocument xmlDoc;
    XMLNode * pRoot = xmlDoc.NewElement("xml");
    xmlDoc.InsertFirstChild(pRoot);
    auto j3 = json::parse(src.c_str());
    for (json::iterator it = j3.begin(); it != j3.end(); ++it) {
        std::string key = it.key();
        std::string value = it.value() ;
        XMLElement * pElement = xmlDoc.NewElement(key.c_str()) ;
        pElement->SetText(value.c_str()) ;
        pRoot->InsertEndChild(pElement);
    }
    XMLPrinter printer;
    pRoot->Accept( &printer );
    return printer.CStr();
}

int main()
{
    std::string src = "<xml>\
				   <appid>appid-value111111</appid>\
				   <mch_id>mch_id-value22222</mch_id>\
				   <nonce_str>nonce_str-value3333333</nonce_str>\
				   <transaction_id>transaction_id-value44444444</transaction_id>\
				   <sign>sign-value5555555555</sign>\
				</xml>" ;
    std::string json = xml2json(src) ;
    std::string xml = json2xml(json) ;


    std::cout <<"json:\n" << json << std::endl ;
    std::cout << "xml:\n" << xml << std::endl ;
}