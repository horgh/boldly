#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <math.h>
#include <float.h>
#include <vector>

using namespace std;

struct Statistic{
    double mean;
    double stddev;
    double n;
    double confl;
    double confr;
    vector<double> values;
};

Statistic getStatistics(char* filename)
{
    FILE * input;
    input = fopen(filename, "r");
    
    Statistic rtn;
    
    double a = 0, b = 0;
    while(fscanf(input, "%lf %lf", &a, &b) != EOF)
    {
        rtn.n++;
        rtn.mean += a;
        rtn.values.push_back(a);
    }
    rtn.mean /= rtn.n;
    
    //calc std dev
    for(vector<double>::iterator it = rtn.values.begin(); it != rtn.values.end(); it++)
        rtn.stddev += ((*it)-rtn.mean)*((*it)-rtn.mean);
    
    rtn.stddev = sqrt( rtn.stddev / (rtn.n - 1) );
    
    //calc 95% confidence interval
    double tValue = 1.833;
    rtn.confl = rtn.mean - (tValue * (rtn.stddev / sqrt(rtn.n)));
    rtn.confr = rtn.mean + (tValue * (rtn.stddev / sqrt(rtn.n)));
    
    return rtn;
}

int main(int argc, char **argv)
{
    if(argc >= 2)
    {
        //read input
        Statistic file1 = getStatistics(argv[1]);
        Statistic file2;
        
        if(argc >= 3)
        {
            file2 = getStatistics(argv[2]);
        }
 
        printf("--------------- File 1 Statistics ---------------\n");
        printf("Data: ");
        for(vector<double>::iterator it = file1.values.begin(); it != file1.values.end(); it++)
            printf("%lf ", (*it));
        printf("\nMean: %lf\n", file1.mean);
        printf("Std Dev: %lf\n", file1.stddev);
        printf("Variance: %lf\n", file1.stddev*file1.stddev);
        printf("95%% Confidence Interval: (%lf, %lf)\n", file1.confl, file1.confr);
        printf("-------------------------------------------------\n");
        if(argc >= 3)
        {
            printf("--------------- File 2 Statistics ---------------\n");
            printf("Data: ");
            for(vector<double>::iterator it = file2.values.begin(); it != file2.values.end(); it++)
                printf("%lf ", (*it));
            printf("\nMean: %lf\n", file2.mean);
            printf("Std Dev: %lf\n", file2.stddev);
            printf("Variance: %lf\n", file2.stddev*file2.stddev);
            printf("95%% Confidence Interval: (%lf, %lf)\n", file2.confl, file2.confr);
            printf("-------------------------------------------------\n");
        }
        
        //assuming normal dist, use p.301 of Devore "Probability and Statistics" - using the null distribution
        double alpha = 0.05;
        int degreesOfFreedom = 9;
        double tReject = -1.833; //pregenerated from book's appendix
        //we are testing against the left side of the confidence interval for file2's mean
        double t = (file1.mean - file2.confl) / (file1.stddev / sqrt(file1.n));

        
        if(argc >= 3)
        {
            printf("H0: mean(file1) = mean(file2) (left side of 95%% conf int. )\n");
            printf("Ha: mean(file1) < mean(file2) (the lowest 'plausible' mean)\n", alpha);
            printf("Sig Level: %lf\n", alpha);
            printf("Degress of Freedom: %d\n", degreesOfFreedom);
            printf("criteria for rejection of H0: t <= %lf\n", tReject);
            printf("t: %lf\n", t);
            printf("Conclusion: %s\n", (t <= tReject? "Reject the null hypothesis. 95% of the time, file1's mean is lower with 0.05 statistical significance." : "Do not reject the null hypothesis."));
        }        
    
        return 0;
    
    }else{
        printf("Usage: analyze <filename>\n\n");
        printf("Note: input file should be the histo output of compute\n");
    }
    
    return 0;
}
