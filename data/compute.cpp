#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <math.h>
#include <float.h>

using namespace std;

inline double dist(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

int main(int argc, char **argv)
{
    if(argc >= 2)
    {
        int minrange = 0;
        int maxrange = 9;
        if(argc >= 3)
            minrange = atoi(argv[2]);
        if(argc >= 4)
            maxrange = atoi(argv[3]);
            
        FILE * positions;
        stringstream posname;
        posname << argv[1] << "/positions";
        positions = fopen(posname.str().c_str(), "r");
        
        //histogram files
        FILE * ratingA, * ratingB, * ratingC;
        stringstream outA, outB, outC;
        outA << argv[1] << "/0-" << argv[1] << ".histo";
        outB << argv[1] << "/1-" << argv[1] << ".histo";
        outC << argv[1] << "/2-" << argv[1] << ".histo";
        ratingA = fopen(outA.str().c_str(), "w");
        ratingB = fopen(outB.str().c_str(), "w");
        ratingC = fopen(outC.str().c_str(), "w");
        
        double throwaway;
        
        for(int i = 0; i < minrange; i++)
            fscanf(positions, "%lf %lf %lf %lf", &throwaway, &throwaway, &throwaway, &throwaway);
    
        for(int i = minrange; i <= maxrange; i++)
        {
            double mapmax;
            fscanf(positions, "%lf %lf %lf %lf", &throwaway, &throwaway, &mapmax, &throwaway);
            
            for(int j = 0; j < 3; j++)
            {
                FILE * file;
                stringstream filename;
                filename << argv[1] << "/" << i << "-" << j << "-" << argv[1];
                file = fopen(filename.str().c_str(), "r");
                bool gotRatingPoint = false;
                
                if(file != NULL)
                {
                    FILE * output;
                    stringstream outname;
                    outname << filename.str() << ".processed";
                    output = fopen(outname.str().c_str(), "w");
                    
                    double travelled = 0.0;
                    double lastx = DBL_MAX;
                    double lasty = DBL_MAX;
                    double time;
                    double maxval;
                    
                    while(!feof(file))
                    {
                        if(lastx == DBL_MAX)
                        {
                            fscanf(file, "%lf %lf %lf %lf", &time, &lastx, &lasty, &maxval);
                            
                        }else{
                            double newx = 0.0;
                            double newy = 0.0;
                            
                            fscanf(file, "%lf %lf %lf %lf", &time, &newx, &newy, &maxval);
                            travelled += dist(lastx, lasty, newx, newy);
                            lastx = newx;
                            lasty = newy;
                        }
                        
                        if(!gotRatingPoint && 100*(maxval / mapmax) >= 95)
                        {
                            if(j == 0)
                                fprintf(ratingA, "%lf 1\n", travelled);
                            else if(j == 1)
                                fprintf(ratingB, "%lf 1\n", travelled);
                            else if(j == 2)
                                fprintf(ratingC, "%lf 1\n", travelled);
                                
                            gotRatingPoint = true;
                        }
                                        
                        fprintf(output, "%lf %lf\n", 100*(maxval / mapmax), travelled);
                    }
                    
                    fclose(output);
                    fclose(file);
                    
                }else{
                    printf("Error: could not open file %s\n", filename.str().c_str());
                    return 2;
                }
            }
        }
        
        return 0;
    }else
        printf("Usage: compute <mapname> [min_range] [max_range]\n");
    
    return 1;
}
