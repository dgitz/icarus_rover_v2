import xml.etree.ElementTree as ET
import os.path
import sys
from datetime import datetime
from array import *

class fieldobject(object):
    def __init__(self,datatype=None,name=None):
        self.datatype=datatype
        self.name=name

def print_usage():
    print "Usage Instructions"
    print "Generate Message: -g <MessageFile.xml>"

def generate_message(xmlfile):
    print "Generating Message files from:",xmlfile
    if(os.path.isfile(xmlfile) == False):
        print "Cannot find file: ",xmlfile, " Exiting."
        sys.exit(0)
    tree = ET.parse(xmlfile)
    root = tree.getroot()

    udpmessagefile_header.write('#ifndef UDPMESSAGE_H\r\n#define UDPMESSAGE_H\r\n')
    udpmessagefile_header.write('#include "ros/ros.h"\r\n#include <icarus_rover_v2/Definitions.h>\r\n#include "ros/time.h"\r\n#include <stdio.h>\r\n')
    udpmessagefile_header.write('#include <iostream>\r\n#include <ctime>\r\n#include <fstream>\r\n#include <iostream>\r\n\r\n')
    for message in root:
        type = message.find('Type').text
        if(type == 'UDP'):
            udpmessagefile_header.write('#define UDP_' + message.get('name') + '_ID ' + message.get('id') +'\r\n')
    udpmessagefile_header.write('\r\nclass UDPMessageHandler\r\n{\r\npublic:\r\n\tUDPMessageHandler();\r\n\t~UDPMessageHandler();\r\n')
    udpmessagefile_cpp.write('#include "udpmessage.h"\r\nUDPMessageHandler::UDPMessageHandler(){}\r\nUDPMessageHandler::~UDPMessageHandler(){}\r\n')
    for message in root:
        print "Message: ",message.get('name'), " id: ", message.get('id')
        type = message.find('Type').text
        fieldlist = []
        fields = message.find('Fields')
        print "Message Type:",type
        for field in fields:
            print "Field date Type: ", field.get('type'), " name: ", field.get('name')
            fieldlist.append(fieldobject(field.get('type'),field.get('name')))
        if(type == 'UDP'):
            udpmessagefile_header.write('\tstd::string encode_' + message.get('name') + 'UDP(')
            udpmessagefile_cpp.write('std::string UDPMessageHandler::encode_' + message.get('name') + 'UDP(')
            index = 0
            for item in fieldlist:
                udpmessagefile_header.write(item.datatype + ' ' + item.name)
                udpmessagefile_cpp.write(item.datatype + ' ' + item.name)
                index += 1
                if(index < len(fieldlist)):
                    udpmessagefile_header.write(',')
                    udpmessagefile_cpp.write(',')
            udpmessagefile_header.write(');\r\n')
            udpmessagefile_cpp.write(')\r\n{\r\n')
            udpmessagefile_cpp.write('\tstd::string tempstr = "";\r\n')
            udpmessagefile_cpp.write('\ttempstr.append(boost::lexical_cast<std::string>(UDP_' + message.get('name') + '_ID));\r\n')
            udpmessagefile_cpp.write('\ttempstr.append(",");\r\n')
            index = 0
            for item in fieldlist:
                if(item.datatype =='std::string'):
                    udpmessagefile_cpp.write('\ttempstr.append(' + item.name +');\r\n')
                else:
                    udpmessagefile_cpp.write('\ttempstr.append(boost::lexical_cast<std::string>(' + item.name + '));\r\n')

                index += 1

                if(index < len(fieldlist)):
                    udpmessagefile_cpp.write('\ttempstr.append(",");\r\n')
            udpmessagefile_cpp.write('\treturn tempstr;\r\n')
            udpmessagefile_cpp.write('}\r\n')
    udpmessagefile_header.write('private:\r\n')
    udpmessagefile_header.write('};\r\n#endif')
    

if len(sys.argv) == 1:
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-g"):
    udpmessagefile_header = open('generated/udpmessage.h','w')
    udpmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    udpmessagefile_header.write('/***Created on:')
    udpmessagefile_header.write( str(datetime.now()))
    udpmessagefile_header.write('***/\r\n')
    udpmessagefile_cpp = open('generated/udpmessage.cpp','w')
    udpmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    udpmessagefile_cpp.write('/***Created on:')
    udpmessagefile_cpp.write( str(datetime.now()))
    udpmessagefile_cpp.write('***/\r\n')
    generate_message(sys.argv[2])





