#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <math.h>
#include <float.h>
#include <vector>

using namespace std;



int main(int argc, char **argv)
{
    if(argc >= 2)
    {
        //read input
        FILE * input;
        input = fopen(argv[1], "r");
        
        double mean2 = 0;
        int n2 = -1;
        if(argc >= 3)
        {
            n2 = 0;
            FILE * input2;
            input2 = fopen(argv[2], "r");
            
            double a = 0, b = 0;
            while(fscanf(input2, "%lf %lf", &a, &b) != EOF)
            {
                n2++;
                mean2 += a;
            }
            mean2 /= n2;
        }
        
        vector<double> data;
        double mean = 0;
        double stddev = 0;
        double n = 0;
        
        double a = 0, b = 0;
        while(fscanf(input, "%lf %lf", &a, &b) != EOF)
        {
            n++;
            mean += a;
            data.push_back(a);
        }
        mean /= n;
        
        //calc std dev
        for(vector<double>::iterator it = data.begin(); it != data.end(); it++)
            stddev += ((*it)-mean)*((*it)-mean);
        
        stddev = sqrt( stddev / (n - 1) );
        
        //calc 95% confidence interval
        double confl = mean - (1.96 * (stddev / n));
        double confr = mean + (1.96 * (stddev / n));
        
        //assuming normal dist, use p.301 of Devore "Probability and Statistics" - using the null distribution
        double alpha = 0.05;
        int degreesOfFreedom = 9;
        double tReject = -1.833; //pregenerated from book's appendix
        double t = (mean - mean2) / (stddev / sqrt(n));
        
        //output
        printf("Data: ");
        for(vector<double>::iterator it = data.begin(); it != data.end(); it++)
            printf("%lf ", (*it));
        printf("\nMean: %lf\n", mean);
        printf("Std Dev: %lf\n", stddev);
        printf("Variance: %lf\n", stddev*stddev);
        
        if(argc >= 3)
        {
            printf("H0: mean(file1) = mean(file2)\n");
            printf("Ha: mean(file1) < mean(file2)\n", alpha);
            printf("Sig Level: %lf\n", alpha);
            printf("Degress of Freedom: %d\n", degreesOfFreedom);
            printf("criteria for rejection of H0: t <= %lf\n", tReject);
            printf("t: %lf\n", t);
            printf("Conclusion: %s\n", (t <= tReject? "Reject the null hypothesis." : "Do not reject the null hypothesis."));
        }
        
        //we don't know the population stddev, only the sample. So, we can't do CIs
        //printf("95%% Confidence Interval: (%lf, %lf)\n", confl, confr);
        
    
        return 0;
    
    }else{
        printf("Usage: analyze <filename>\n\n");
        printf("Note: input file should be the histo output of compute\n");
    }
    
    return 0;
}
