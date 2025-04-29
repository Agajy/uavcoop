#include <Matrix.h>
#include <Vector2D.h>
#include <Vector3D.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <fstream>
#include <sstream>
#include <cmath>

class Trajectory {

    public:
        std::vector<std::vector<float>> array;

        int index;
        bool first_it;
        flair::core::Time t0;
        int N;
        double t_curr_prev;
        double curr_dist;

        Trajectory(){
            std::cout << "Reading path data...\n";
            this->getPathData();
            std::cout << "...data loaded!\n";
            reset();
        }

        void reset(){
            index = 0;
            first_it = true;
            t_curr_prev = 0;
            curr_dist = 0;
        }

        void get_point(const flair::core::Time& t1, flair::core::Vector2Df& pos, flair::core::Vector2Df& veloc, float& angle, const float& vel){

            if (first_it){
                t0 = t1;
                first_it = false;
                }

            double t_curr = (t1-t0)*1e-9;
            double dt = t_curr - t_curr_prev;
            t_curr_prev = t_curr;

            // float vel = 0.26;
            float segment = 0.15; //0.03
            // float xxx = array.at(index+1).at(0)-array.at(index).at(0);
            // float yyy = array.at(index+1).at(1)-array.at(index).at(1);
            // float aaaa = std::sqrt(xxx*xxx + yyy*yyy);
            // std::cout << "dist: " << aaaa<< std::endl;
            index = (int)std::floor(curr_dist + vel*dt/segment);
            // index = (int)std::floor(vel*t_curr/segment);
            if (index>=N-1){
                pos.x = array.at(N-1).at(0);
                pos.y = array.at(N-1).at(1);
                angle = array.at(N-1).at(2);
                veloc.x = 0;
                veloc.y = 0;
            }
            else{
                curr_dist = curr_dist + vel*dt/segment;
                float factor = curr_dist - (float)index;
                // std::cout << "factor: " << factor << std::endl;
                pos.x = array.at(index).at(0)*(1.-factor) + factor*(array.at(index+1).at(0));
                pos.y = array.at(index).at(1)*(1.-factor) + factor*(array.at(index+1).at(1));
                angle = array.at(index).at(2)*(1.-factor) + factor*(array.at(index+1).at(2));
                veloc.x = std::cos(angle)*vel;
                veloc.y = std::sin(angle)*vel;
                }
            // angle = 0;
            angle = atan2(std::sin(angle),std::cos(angle));
        }
        // N = 750#750


        void get_point_old(const flair::core::Time& t1, flair::core::Vector2Df& pos, float& angle){
            pos.x = array.at(index).at(1);
            pos.y = array.at(index).at(2);
            angle = array.at(index).at(3);

            if (first_it){
                t0 = t1;
                first_it = false;
                }

            double t_curr = (t1-t0)*1e-9;

            // std::cout << t_curr << ", " << array.at(N-1).at(0) << ", " << index << ", " << N-1 << std::endl;
            if ((t_curr >= array.at(N-1).at(0)) || (index>N-1))
            {
                // std::cout << "111111" << std::endl;
                pos.x = array.at(N-1).at(1);
                pos.y = array.at(N-1).at(2);
                // return;
            }
            else {
                for (size_t j = index; j < N-1; j++)
                {
                    if ((t_curr<array.at(j+1).at(0)) && (t_curr>=array.at(j).at(0))){
                        // std::cout << ":: " << out.x << ", " << out.y << std::endl;
                        float factor = (t_curr-array.at(j).at(0))/(array.at(j+1).at(0)-array.at(j).at(0));
                        index = j;
                        pos.x = array.at(j).at(1) + factor*(array.at(j+1).at(1)-array.at(j).at(1));
                        pos.y = array.at(j).at(2) + factor*(array.at(j+1).at(2)-array.at(j).at(2));
                        break;
                    }
                }
            }
            // std::cout << "desired pos: " << t_curr << ", " << out.x << ", " << out.y << std::endl;
        }
        // N = 750#750
        // if t_curr >= self.t[N-1] or self.i>N-1:
        //     return self.x[N-1], self.y[N-1], True
        // else:
        //     for j in range(self.i,N-1):
        //         if t_curr<self.t[j+1] and t_curr>=self.t[j]:
        //             factor = (t_curr-self.t[j])/(self.t[j+1]-self.t[j])
        //             self.i = j
        //             return self.x[j] + factor*(self.x[j+1]-self.x[j]), self.y[j] + factor*(self.y[j+1]-self.y[j]), True
        //     return 0,0, False


    private:
        void getPathData()
        {
            std::ifstream  data;
            data.open("./path.txt");
            std::string line;
            bool firstline = true;
            while(std::getline(data,line))
            {
                if (firstline){
                    firstline = false;
                    // continue;
                    }
                std::string val;
                std::vector<float> v(3);    
                // v.resize(3*sizeof(float));             /* row vector v */
                std::stringstream s(line);         /* stringstream line */
                // while (getline (s, val, ';'))       /* get each value (',' delimited) */
                float x, y;
                for (size_t i = 0; i < 3; i++) // x y angle
                {
                    getline (s, val, ';');
                    v.at(i) = std::stof(val);  /* add to row vector */
                    if(i==1)
                        x = v.at(i);
                    if(i==2)
                        y = v.at(i);
                }
                v.at(1) = x;
                v.at(2) = y;
                // v.at(3) = x; // orientation
                array.push_back (v);  
            }
            N = array.size();
            // array.resize(N);

            // for (size_t i = 0; i < N; i++)
            // {
            //     array.at(i).at(0) = array.at(i).at(0) - array.at(0).at(0);
            //     // std::cout << "t: " << array.at(i).at(0) << std::endl;
            // }
            
            // for (auto& row : array) {               /* iterate over rows */
            //     for (auto& val : row)               /* iterate over vals */
            //         std::cout << val << "  ";       /* output value      */
            //     std::cout << "\n";                  /* tidy up with '\n' */
            // }
            // std::cout << array.size() << std::endl;
            // std::cout << array.at(0).at(0) << ", " << array.at(0).at(1) << ", " << array.at(0).at(2) << std::endl;
        }
};