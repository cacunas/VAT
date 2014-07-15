#include "supervisedstatisticalinfo.h"

SupervisedStatisticalInfo::SupervisedStatisticalInfo(): mean(0.0),
                                                        sd(0.0),
                                                        median(0.0),
                                                        R(0.0),
                                                        pca(NULL) {}

SupervisedStatisticalInfo::SupervisedStatisticalInfo(float m, float SD, float M, float r) {
    init(m, SD, M, r);
}

SupervisedStatisticalInfo::~SupervisedStatisticalInfo() {
}

void SupervisedStatisticalInfo::init(float mean, float sd, float median, float R) {
    this->mean = mean;
    this->sd = sd;
    this->median = median;
    this->R = R;
    Image = NULL;
    pca = NULL;
}

std::ostream& operator<<(std::ostream& out, const SupervisedStatisticalInfo& sup) {

    out << "\tMean: \t" << sup.mean << std::endl;
    out << "\tMedian: \t" << sup.median << std::endl;
    out << "\tSt. Dev.:\t" << sup.sd << std::endl;
    out << "\tReliability: " << sup.R << std::endl;
    out << "\tDescriptor: " << std::endl << "\t\t[ ";
    int i, s = sup.descriptor.size();
    for(i=0; i<s; i++){
        if(i != s-1)
            out << sup.descriptor[i] << "; ";
        else
            out << sup.descriptor[i] << " ]" << std::endl;
    }
    return out;
}
