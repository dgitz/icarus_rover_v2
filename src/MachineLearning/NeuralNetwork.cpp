#include "NeuralNetwork.h"
NeuralNetwork::NeuralNetwork(icarus_rover_v2::diagnostic diag)
{
    diagnostic = diag;
    type = "Raw";
    layercount = 0;
}
NeuralNetwork::~NeuralNetwork()
{

}
icarus_rover_v2::diagnostic NeuralNetwork::loadNetworkfile(std::string name)
{
    layers.clear();
    icarus_rover_v2::diagnostic diag = diagnostic;
    struct stat buffer; 
    std::string filepath = "/home/robot/config/NeuralNets/" + name + ".xml";
    if(stat (filepath.c_str(), &buffer) != 0)
    {
        diag.Diagnostic_Type = DATA_STORAGE;
        diag.Level = ERROR;
        diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
        char tempstr[512];
        sprintf(tempstr,"Neural Network at path: %s Does not Exist.",filepath.c_str());
        diag.Description = std::string(tempstr);
        return diag;
    }
    TiXmlDocument doc(filepath.c_str());
    bool file_loaded = doc.LoadFile();
    if(file_loaded == true)
    {
    	if(parse_Networkfile(doc) == true)
        {
            diag.Diagnostic_Type = DATA_STORAGE;
            diag.Level = INFO;
            diag.Diagnostic_Message = NOERROR;
            char tempstr[512];
            sprintf(tempstr,"Loaded Neural Network at path: %s.",filepath.c_str());
            diag.Description = std::string(tempstr);
            return diag;
        }
        else
        {
            diag.Diagnostic_Type = DATA_STORAGE;
            diag.Level = ERROR;
            diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
            char tempstr[512];
            sprintf(tempstr,"Couldn't Parse Neural Network at path: %s.",filepath.c_str());
            diag.Description = std::string(tempstr);
            return diag;
        }
    }
    else
    {
        diag.Diagnostic_Type = DATA_STORAGE;
        diag.Level = ERROR;
        diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
        char tempstr[512];
        sprintf(tempstr,"Couldn't Load Neural Network at path: %s.",filepath.c_str());
        diag.Description = std::string(tempstr);
        return diag;
    }
}
std::vector<double> NeuralNetwork::evaluate_network(std::vector<double> input_vector)
{
    std::vector<double> input;
    std::vector<double> output;
    for(int i = 0; i < layers.size(); i++)
    {
        input.clear();
        if(i == 0)
        {
            input = input_vector;
        }
        else
        {
            for(int j = 0; j < layers.at(i-1).neurons.size(); j++)
            {
                input.push_back(layers.at(i-1).neurons.at(j).output);
            }
   
        }
        for(int j = 0; j < layers.at(i).neurons.size(); j++)
        {
            layers.at(i).neurons.at(j).input.clear();
            layers.at(i).neurons.at(j).input = input;
            double z = 0;
            for(int k = 0; k < layers.at(i).neurons.at(j).input.size(); k++)
            {
                z += layers.at(i).neurons.at(j).input.at(k)*layers.at(i).neurons.at(j).weight.at(k);
            }
            if(activation_function == "tanh")
            {
                layers.at(i).neurons.at(j).output = tanh(z);
            }
        }
    }
    output.clear();
    for(int j = 0; j < layers.at(layers.size()-1).neurons.size(); j++)
    {
        output.push_back(layers.at(layers.size()-1).neurons.at(j).output);
    }
    return output;
}
bool NeuralNetwork::parse_Networkfile(TiXmlDocument doc)
{
    TiXmlElement *l_pRootElement = doc.RootElement();
	if( NULL != l_pRootElement )
	{
        TiXmlElement *l_pInfo = l_pRootElement->FirstChildElement( "Info" );

        if ( NULL != l_pInfo )
        {
                TiXmlElement *l_pType = l_pInfo->FirstChildElement("Type");
                if ( NULL != l_pType )
                {
                    type = l_pType->GetText();
                }
                else { return false; }
                TiXmlElement *l_pActivationFunction = l_pInfo->FirstChildElement("ActivationFunction");
                if ( NULL != l_pActivationFunction )
                {
                    activation_function = l_pActivationFunction->GetText();
                }
                else { return false; }
        }
        else { return false; }
        TiXmlElement *l_pLayerCount = l_pRootElement->FirstChildElement( "LayerCount" );
        if ( NULL != l_pLayerCount)
        {
            layercount = atoi(l_pLayerCount->GetText());
        }
        else { return false; }
        TiXmlElement *l_pLayer = l_pRootElement->FirstChildElement( "Layer" );
        while(l_pLayer)
        {
            Layer newlayer;
            
            TiXmlElement *l_pName = l_pLayer->FirstChildElement( "Name" );
            if ( NULL != l_pLayer)
            {
                newlayer.name = l_pName->GetText();
            }
            else { return false; }
           
            TiXmlElement *l_pNeuronCount = l_pLayer->FirstChildElement( "NeuronCount" );
            if ( NULL != l_pNeuronCount)
            {
                newlayer.neuroncount = atoi(l_pNeuronCount->GetText());
            }
            else { return false; }
            
            TiXmlElement *l_pNeuron = l_pLayer->FirstChildElement("Neuron");
            while(l_pNeuron)
            {
                Neuron newneuron;
                newneuron.output = 0.0;
                TiXmlElement *l_pInputCount = l_pNeuron->FirstChildElement("InputCount");
                if (NULL != l_pInputCount)
                {
                    newneuron.inputcount = atoi(l_pInputCount->GetText());
                }
                else { return false; }
                
                for(int i = 0; i < newneuron.inputcount; i++)
                {
                    newneuron.input.push_back(0.0);
                }
                
                std::string tempstr_weight;
                TiXmlElement *l_pInputWeight = l_pNeuron->FirstChildElement("InputWeight");
                if (NULL != l_pInputWeight)
                {
                    tempstr_weight = l_pInputWeight->GetText();
                }
                else { return false; }
                std::vector<std::string> strs;
                boost::split(strs,tempstr_weight,boost::is_any_of(","));
                for(int i = 0; i < strs.size(); i++)
                {
                    newneuron.weight.push_back(atof(strs.at(i).c_str()));
                }
                if(newneuron.weight.size() != newneuron.input.size()) { return false; }
                newlayer.neurons.push_back(newneuron);
                l_pNeuron = l_pNeuron->NextSiblingElement( "Neuron" );
            }
            if(newlayer.neuroncount == 0) { return false; }
            if(newlayer.neuroncount != newlayer.neurons.size()) { return false; }
            
            layers.push_back(newlayer);
            l_pLayer = l_pLayer->NextSiblingElement( "Layer" );
        }
        if(layercount == 0) { return false; }
        if(layercount != layers.size()) { return false; }
        
        TiXmlElement *l_pSampleData = l_pRootElement->FirstChildElement( "SampleData" );
        if ( NULL != l_pSampleData)
        {
            TiXmlElement *l_pInput = l_pSampleData->FirstChildElement("Input");
            while(l_pInput)
            {
                TiXmlElement *l_pInputData = l_pInput->FirstChildElement( "Data" );
                if ( NULL != l_pInputData)
                {
                    std::vector<double> input_data;
                    std::string tempstr_data = l_pInputData->GetText();
                    std::vector<std::string> strs;
                    boost::split(strs,tempstr_data,boost::is_any_of(","));
                    for(int i = 0; i < strs.size(); i++)
                    {
                        input_data.push_back(atof(strs.at(i).c_str()));
                    }
                    sample_inputdata.push_back(input_data);
                    
                }
                else { return false;}
                
                l_pInput = l_pInput->NextSiblingElement( "Input" );
            }
            
            TiXmlElement *l_pOutput = l_pSampleData->FirstChildElement("Output");
            while(l_pOutput)
            {
                TiXmlElement *l_pOutputData = l_pOutput->FirstChildElement( "Data" );
                if ( NULL != l_pOutputData)
                {
                    std::vector<double> output_data;
                    std::string tempstr_data = l_pOutputData->GetText();
                    std::vector<std::string> strs;
                    boost::split(strs,tempstr_data,boost::is_any_of(","));
                    for(int i = 0; i < strs.size(); i++)
                    {
                        output_data.push_back(atof(strs.at(i).c_str()));
                    }
                    sample_outputdata.push_back(output_data);       
                }
                else { return false;}
                
                l_pOutput = l_pOutput->NextSiblingElement( "Output" );
            }
        }
        else { return false; }
    }
    else { return false; }
    return true;
}