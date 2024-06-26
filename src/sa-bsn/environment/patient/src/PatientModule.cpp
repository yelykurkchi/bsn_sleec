#include <PatientModule.hpp>

PatientModule::PatientModule(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name) {}

PatientModule::~PatientModule() {}

void PatientModule::setUp() {
    srand(time(NULL));

    // TODO change Operation to static
    std::string vitalSigns;
    std::string otherSensors;
    service = nh.advertiseService("getPatientData", &PatientModule::getPatientData, this);
    double aux;

    frequency = 1000;

    // Get what vital signs this module will simulate
    nh.getParam("vitalSigns", vitalSigns);

    // Removes white spaces from vitalSigns
    vitalSigns.erase(std::remove(vitalSigns.begin(), vitalSigns.end(),' '), vitalSigns.end());

    std::vector<std::string> splittedVitalSigns = bsn::utils::split(vitalSigns, ',');

    for (std::string s : splittedVitalSigns) {
        vitalSignsFrequencies[s] = 0;
        nh.getParam(s + "_Change", aux);
        vitalSignsChanges[s] = 1/aux;
        nh.getParam(s + "_Offset", vitalSignsOffsets[s]);
    }

    for (const std::string& s : splittedVitalSigns) {
        patientData[s] = configureVitalSignDataGenerator(s);
    }
    
    // Get what other sensors this module will simulate
    nh.getParam("otherSensors", otherSensors);
    gpsFrequency = 0;
    nh.getParam("gps_Change", aux);
    gpsChanges = 1/aux;
    nh.getParam("gps_Offset", gpsOffset);

    gpsData = configureGPSDataGenerator("gps");

    rosComponentDescriptor.setFreq(frequency);
    
    period = 1/frequency;
}

bsn::generator::DataGenerator PatientModule::configureVitalSignDataGenerator(const std::string& vitalSign) {
    srand(time(NULL));
    
    std::vector<std::string> t_probs;
    std::array<float, 25> transitions;
    std::array<bsn::range::Range,5> ranges;
    std::string s;
    ros::NodeHandle handle;

    for(uint32_t i = 0; i < transitions.size(); i++){
        for(uint32_t j = 0; j < 5; j++){
            handle.getParam(vitalSign + "_State" + std::to_string(j), s);
            t_probs = bsn::utils::split(s, ',');
            for(uint32_t k = 0; k < 5; k++){
                transitions[i++] = std::stod(t_probs[k]);
            }
        }
    }
    
    std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

    handle.getParam(vitalSign + "_LowRisk", s);
    lrs = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_MidRisk0", s);
    mrs0 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_HighRisk0", s);
    hrs0 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_MidRisk1", s);
    mrs1 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_HighRisk1", s);
    hrs1 = bsn::utils::split(s, ',');

    ranges[0] = bsn::range::Range(std::stod(hrs0[0]), std::stod(hrs0[1]));
    ranges[1] = bsn::range::Range(std::stod(mrs0[0]), std::stod(mrs0[1]));
    ranges[2] = bsn::range::Range(std::stod(lrs[0]), std::stod(lrs[1]));
    ranges[3] = bsn::range::Range(std::stod(mrs1[0]), std::stod(mrs1[1]));
    ranges[4] = bsn::range::Range(std::stod(hrs1[0]), std::stod(hrs1[1]));

    bsn::generator::Markov markov(transitions, ranges, 2);
    bsn::generator::DataGenerator dataGenerator(markov);
    dataGenerator.setSeed();

    return dataGenerator;
}

bsn::generator::DataGenerator PatientModule::configureGPSDataGenerator(const std::string& gps) {
    srand(time(NULL));
    
    std::vector<std::string> t_probs;
    std::array<float, 25> transitions;
    std::array<std::string,5> locations;
    std::string s;
    ros::NodeHandle handle;

    for(uint32_t i = 0; i < transitions.size(); i++){
        for(uint32_t j = 0; j < 5; j++){
            handle.getParam(gps + "_State" + std::to_string(j), s);
            t_probs = bsn::utils::split(s, ',');
            for(uint32_t k = 0; k < 5; k++){
                transitions[i++] = std::stod(t_probs[k]);
            }
        }
    }
    
    for(uint32_t i = 0; i < locations.size(); i++){
        locations[i] = handle.getParam(gps + "_loc" + std::to_string(i), s);
    }

    bsn::generator::Markov markov(transitions, locations, 2);
    bsn::generator::DataGenerator dataGenerator(markov);
    dataGenerator.setSeed();

    return dataGenerator;
}

void PatientModule::tearDown() {}

bool PatientModule::getPatientData(services::PatientData::Request &request, 
                                services::PatientData::Response &response) {
    
    if(request.vitalSign=="gps"){
        response.data =  gpsData.getValue();
    } else {
        response.data = patientData[request.vitalSign].getValue();
    }
    
    ROS_INFO("Answered a request for %s's data.", request.vitalSign.c_str());

    return true;
}

void PatientModule::body() {
    // Loops over all of the sensors and changes the states with time (frequency, offset)
    for (auto &p : vitalSignsFrequencies) {
        
        if (p.second >= (vitalSignsChanges[p.first] + vitalSignsOffsets[p.first])) {
            patientData[p.first].nextState();
            p.second = vitalSignsOffsets[p.first];
            ROS_DEBUG("Transitioned %s's state", p.first.c_str());
        } else {
            p.second += period;
        }
    }
    if (gps_Frequency >= gpsChanges + gpsOffset){
        gpsData.nextState();
        gps_Frequency = gpsOffset;
        ROS_DEBUG("Transitioned GPS' state");
    } else {
        gps_Frequency += period;
    }
}

