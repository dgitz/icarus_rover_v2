import xml.etree.ElementTree as ET
import os.path
import sys
from datetime import datetime
from array import *

class fieldobject(object):
    def __init__(self,datatype=None,name=None):
        self.datatype=datatype
        self.name=name
class protocolobject(object):
    def __init__(self,name=None):
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
    udpmessagefile_header.write('#include "ros/ros.h"\r\n#include "Definitions.h"\r\n#include "ros/time.h"\r\n#include <stdio.h>\r\n')
    udpmessagefile_header.write('#include <iostream>\r\n#include <ctime>\r\n#include <fstream>\r\n#include <iostream>\r\n\r\n')
    ros_serialmessagefile_header.write('#ifndef SERIALMESSAGE_H\r\n#define SERIALMESSAGE_H\r\n')
    propeller_serialmessagefile_header.write('#ifndef SERIALMESSAGE_H\r\n#define SERIALMESSAGE_H\r\n')
    for message in root:
        protocollist = []
        protocols = message.find('Protocols')
        for protocol in protocols:
            #print "Protocol: ", protocol.get('name')
            protocollist.append(protocolobject(protocol.get('name')))
            if(protocol.get('name') == 'UDP'):
                udpmessagefile_header.write('#define UDP_' + message.get('name') + '_ID ' + message.get('id') +'\r\n')
            if(protocol.get('name') == 'Serial'):
                ros_serialmessagefile_header.write('#define SERIAL_' + message.get('name') + '_ID ' + hex(int(message.get('id'),0)-int('0xAB00',0)) + '\r\n')
                propeller_serialmessagefile_header.write('#define SERIAL_' + message.get('name') + '_ID ' + hex(int(message.get('id'),0)-int('0xAB00',0)) + '\r\n')
    udpmessagefile_header.write('\r\nclass UDPMessageHandler\r\n{\r\npublic:\r\n\tUDPMessageHandler();\r\n\t~UDPMessageHandler();\r\n')
    udpmessagefile_cpp.write('#include "udpmessage.h"\r\nUDPMessageHandler::UDPMessageHandler(){}\r\nUDPMessageHandler::~UDPMessageHandler(){}\r\n')
    ros_serialmessagefile_header.write('\r\nclass SerialMessageHandler\r\n{\r\npublic:\r\n\tSerialMessageHandler();\r\n\t~SerialMessageHandler();\r\n')
    ros_serialmessagefile_cpp.write('#include "serialmessage.h"\r\nSerialMessageHandler::SerialMessageHandler(){}\r\nSerialMessageHandler::~SerialMessageHandler(){}\r\n')
    propeller_serialmessagefile_cpp.write('#include "serialmessage.h"\r\n')
    for message in root:
        protocollist = []
        protocols = message.find('Protocols')
        for protocol in protocols:
            fieldlist = []
            fields = protocol.find('Fields')
            #print fields
            for field in fields:
                #print "Field date Type: ", field.get('type'), " name: ", field.get('name')
                fieldlist.append(fieldobject(field.get('type'),field.get('name')))
           
            if(protocol.get('name') == 'UDP'):
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
                        udpmessagefile_cpp.write('\ttempstr.append(boost::lexical_cast<std::string>((int)' + item.name + '));\r\n')

                    index += 1

                    if(index < len(fieldlist)):
                        udpmessagefile_cpp.write('\ttempstr.append(",");\r\n')
                udpmessagefile_cpp.write('\treturn tempstr;\r\n')
                udpmessagefile_cpp.write('}\r\n')
            elif(protocol.get('name') == 'Serial'):
                encode_for_master = 0
                encode_for_slave = 0
                decode_for_master = 0
                decode_for_slave = 0
                for child in protocol.findall('Origin'):
                    if(child.text == "Master"):
                        encode_for_master = 1
                        decode_for_slave = 1
                    if(child.text == "Slave"):
                        decode_for_master = 1
                        encode_for_slave = 1
                #print message.get('name'), " Encoding/Decoding for Master: ", encode_for_master, ",", decode_for_master, " Encoding/Decoding for Slave: ", encode_for_slave, ",", decode_for_slave 
                if(encode_for_master == 1):                
                    ros_serialmessagefile_header.write('\tint encode_' + message.get('name') + 'Serial(unsigned char* outbuffer,int* length,')
                    ros_serialmessagefile_cpp.write('int SerialMessageHandler::encode_' + message.get('name') + 'Serial(unsigned char* outbuffer,int* length,')
                if(encode_for_slave == 1):
                    propeller_serialmessagefile_header.write('int encode_' + message.get('name') + 'Serial(int* outbuffer,int* length,')
                    propeller_serialmessagefile_cpp.write('int encode_' + message.get('name') + 'Serial(int* outbuffer,int* length,')
                index = 0
                for item in fieldlist:
                    if(encode_for_master == 1):
                        if(item.datatype == 'char'):
                            ros_serialmessagefile_header.write(item.datatype + ' ' + item.name)
                            ros_serialmessagefile_cpp.write(item.datatype + ' ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            ros_serialmessagefile_header.write('int ' + item.name)
                            ros_serialmessagefile_cpp.write('int ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype
                    if(encode_for_slave == 1):
                        if(item.datatype == 'char'):
                            propeller_serialmessagefile_header.write(item.datatype + ' ' + item.name)
                            propeller_serialmessagefile_cpp.write(item.datatype + ' ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            propeller_serialmessagefile_header.write('int ' + item.name)
                            propeller_serialmessagefile_cpp.write('int ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype
                    index += 1
                    if(index < len(fieldlist)):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_header.write(',')
                            ros_serialmessagefile_cpp.write(',')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_header.write(',')
                            propeller_serialmessagefile_cpp.write(',')
                if(encode_for_master == 1):
                    ros_serialmessagefile_header.write(');\r\n')
                    ros_serialmessagefile_cpp.write(')\r\n{\r\n')
                if(encode_for_slave == 1):
                    propeller_serialmessagefile_header.write(');\r\n')
                    propeller_serialmessagefile_cpp.write(')\r\n{\r\n')
                message_id = hex(int(message.get('id'),0)-int('0xAB00',0))
                bytelength = 0
                for item in fieldlist:
                    if(item.datatype == 'char'):
                        bytelength = bytelength +1
                    elif(item.datatype == 'uint16_t'):
                        bytelength = bytelength +2
                    else:
                        print "ERROR: Datatype not supported:",item.datatype
                if(bytelength > 8): print "ERROR ERROR ERROR: Currently Serial Messages longer than 8 bytes are not supported."
                if(encode_for_master == 1):
                    ros_serialmessagefile_cpp.write('\tunsigned char *p_outbuffer;\r\n\tp_outbuffer = &outbuffer[0];\r\n')
                    ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = 0xAB;\r\n')
                    ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + message_id +';\r\n')
                    ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = 8;\r\n')
                if(encode_for_slave == 1):
                    propeller_serialmessagefile_cpp.write('\tint byte_counter=0;\r\n')
                    propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = 0xAB;\r\n')
                    propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = ' + message_id +';\r\n')
                    propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = 8;\r\n')
                for item in fieldlist:
                    if(item.datatype == 'char'):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = ' + item.name +';\r\n')
                    elif(item.datatype == 'uint16_t'):
                        if(encode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = ' + item.name +';\r\n')
                        if(encode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '1 = ' + item.name + ' >> 8;\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'1;\r\n')
                            propeller_serialmessagefile_cpp.write('\tint v_' + item.name + '2 = ' + item.name + ' -(v_'  + item.name + '1 << 8);\r\n')
                            propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = v_' + item.name +'2;\r\n')
                    else:
                        print "ERROR: Datatype not supported:",item.datatype
                for b in range(bytelength,8):
                    if(encode_for_master == 1):
                        ros_serialmessagefile_cpp.write('\t*p_outbuffer++ = 0;\r\n')
                    if(encode_for_slave == 1):
                        propeller_serialmessagefile_cpp.write('\toutbuffer[byte_counter++] = 0;\r\n') 
                if(encode_for_master == 1):
                    ros_serialmessagefile_cpp.write('\tint checksum = 0;\r\n')
                    ros_serialmessagefile_cpp.write('\tfor(int i = 3; i < (3+8);i++)\r\n\t{\r\n')
                    ros_serialmessagefile_cpp.write('\t\tchecksum ^= outbuffer[i];\r\n')
                    ros_serialmessagefile_cpp.write('\t}\r\n\t*p_outbuffer++ = checksum;\r\n\t*length = p_outbuffer-&outbuffer[0];\r\n')
                    ros_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    ros_serialmessagefile_cpp.write('}\r\n')
                if(encode_for_slave == 1):
                    propeller_serialmessagefile_cpp.write('\tint checksum = 0;\r\n')
                    propeller_serialmessagefile_cpp.write('\tfor(int i = 3; i < (3+8);i++)\r\n\t{\r\n')
                    propeller_serialmessagefile_cpp.write('\t\tchecksum ^= outbuffer[i];\r\n')
                    propeller_serialmessagefile_cpp.write('\t}\r\n\toutbuffer[byte_counter] = checksum;\r\n\tlength[0] = 3+8+1;\r\n')
                    propeller_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    propeller_serialmessagefile_cpp.write('}\r\n')
                
                if(decode_for_master == 1):
                    ros_serialmessagefile_header.write('\tint decode_' + message.get('name') + 'Serial(unsigned char* inpacket,')
                    ros_serialmessagefile_cpp.write('int SerialMessageHandler::decode_' + message.get('name') + 'Serial(unsigned char* inpacket,')
                if(decode_for_slave == 1):
                    propeller_serialmessagefile_header.write('int decode_' + message.get('name') + 'Serial(int* inpacket,int length,int checksum,')
                    propeller_serialmessagefile_cpp.write('int decode_' + message.get('name') + 'Serial(int* inpacket,int length,int checksum,')
                index = 0
                for item in fieldlist:
                    if(decode_for_master == 1):
                        if(item.datatype == 'char'):
                            ros_serialmessagefile_header.write(item.datatype + '* ' + item.name)
                            ros_serialmessagefile_cpp.write(item.datatype + '* ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            ros_serialmessagefile_header.write('int* ' + item.name)
                            ros_serialmessagefile_cpp.write('int* ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype
                    if(decode_for_slave == 1):
                        if(item.datatype == 'char'):
                            propeller_serialmessagefile_header.write(item.datatype + '* ' + item.name)
                            propeller_serialmessagefile_cpp.write(item.datatype + '* ' + item.name)
                        elif(item.datatype == 'uint16_t'):
                            propeller_serialmessagefile_header.write('int* ' + item.name)
                            propeller_serialmessagefile_cpp.write('int* ' + item.name)
                        else:
                            print "ERROR: Datatype not supported:",item.datatype
                    index += 1
                    if(index < len(fieldlist)):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_header.write(',')
                            ros_serialmessagefile_cpp.write(',')
                        if(decode_for_slave == 1):
                            propeller_serialmessagefile_header.write(',')
                            propeller_serialmessagefile_cpp.write(',')
                if(decode_for_master == 1):
                    ros_serialmessagefile_header.write(');\r\n')
                    ros_serialmessagefile_cpp.write(')\r\n{\r\n')
                if(decode_for_slave == 1):
                    propeller_serialmessagefile_header.write(');\r\n')
                    propeller_serialmessagefile_cpp.write(')\r\n{\r\n')
                    propeller_serialmessagefile_cpp.write('\tint computed_checksum = 0;\r\n')
                    propeller_serialmessagefile_cpp.write('\tfor(int i = 0; i < length; i++)\r\n\t{\r\n')
                    propeller_serialmessagefile_cpp.write('\t\tcomputed_checksum ^= inpacket[i];\r\n\t}\r\n')
                    propeller_serialmessagefile_cpp.write('\tif(computed_checksum != checksum) { return -1; }\r\n')
                bytecounter = 0
                for item in fieldlist:
                    if(item.datatype == 'char'):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter) + '];\r\n')
                        if(decode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter) + '];\r\n')
                        bytecounter = bytecounter + 1
                    elif(item.datatype == 'uint16_t'):
                        if(decode_for_master == 1):
                            ros_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            ros_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter) + '] + v_' + str(item.name) + '1;\r\n')
                            bytecounter = bytecounter + 1
                        elif(decode_for_slave == 1):
                            propeller_serialmessagefile_cpp.write('\tint v_' + str(item.name) + '1=inpacket[' + str(bytecounter) + ']<<8;\r\n')
                            bytecounter = bytecounter + 1
                            propeller_serialmessagefile_cpp.write('\t*' + str(item.name) + '=inpacket[' + str(bytecounter) + '] + v_' + str(item.name) + '1;\r\n')
                            bytecounter = bytecounter + 1
                    else:
                        print "ERROR: Datatype not supported:",item.datatype
                if(decode_for_master == 1):
                    ros_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    ros_serialmessagefile_cpp.write('}\r\n')
                if(decode_for_slave == 1):
                    propeller_serialmessagefile_cpp.write('\treturn 1;\r\n')
                    propeller_serialmessagefile_cpp.write('}\r\n')
    udpmessagefile_header.write('private:\r\n')
    udpmessagefile_header.write('};\r\n#endif')
    ros_serialmessagefile_header.write('private:\r\n')
    ros_serialmessagefile_header.write('};\r\n#endif')
    propeller_serialmessagefile_header.write('#endif')



if len(sys.argv) == 1:
    print_usage()
    sys.exit(0)
elif (sys.argv[1] == "-g"):
    udpmessagefile_header = open('generated/ros/udpmessage.h','w')
    udpmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    udpmessagefile_header.write('/***Created on:')
    udpmessagefile_header.write( str(datetime.now()))
    udpmessagefile_header.write('***/\r\n')
    udpmessagefile_cpp = open('generated/ros/udpmessage.cpp','w')
    udpmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    udpmessagefile_cpp.write('/***Created on:')
    udpmessagefile_cpp.write( str(datetime.now()))
    udpmessagefile_cpp.write('***/\r\n')

    ros_serialmessagefile_header = open('generated/ros/serialmessage.h','w')
    ros_serialmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_serialmessagefile_header.write('/***Created on:')
    ros_serialmessagefile_header.write( str(datetime.now()))
    ros_serialmessagefile_header.write('***/\r\n')
    ros_serialmessagefile_header.write("/***Target: Raspberry Pi ***/\r\n")
    ros_serialmessagefile_cpp = open('generated/ros/serialmessage.cpp','w')
    ros_serialmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    ros_serialmessagefile_cpp.write('/***Created on:')
    ros_serialmessagefile_cpp.write( str(datetime.now()))
    ros_serialmessagefile_cpp.write('***/\r\n')
    ros_serialmessagefile_cpp.write("/***Target: Raspberry Pi ***/\r\n")

    propeller_serialmessagefile_header = open('generated/propeller/serialmessage.h','w')
    propeller_serialmessagefile_header.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    propeller_serialmessagefile_header.write('/***Created on:')
    propeller_serialmessagefile_header.write( str(datetime.now()))
    propeller_serialmessagefile_header.write('***/\r\n')
    propeller_serialmessagefile_header.write("/***Target: Parallax Propeller ***/\r\n")
    propeller_serialmessagefile_cpp = open('generated/propeller/serialmessage.c','w')
    propeller_serialmessagefile_cpp.write('/***************AUTO-GENERATED.  DO NOT EDIT********************/\r\n')
    propeller_serialmessagefile_cpp.write('/***Created on:')
    propeller_serialmessagefile_cpp.write( str(datetime.now()))
    propeller_serialmessagefile_cpp.write('***/\r\n')
    propeller_serialmessagefile_cpp.write("/***Target: Parallax Propeller ***/\r\n")
    generate_message(sys.argv[2])





