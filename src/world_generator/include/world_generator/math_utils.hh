#pragma once 
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Rand.hh>


namespace math_utils{

template <class T>
struct Tuple{
    T r;
    T c;

    Tuple(T r, T c){
        this->r = r;
        this->c = c;
    }

    Tuple(){

    }

};

ignition::math::Vector3d RandomPointInCircle(double radius){
    double t = 2*3.141*ignition::math::Rand::DblUniform(0,1);
    double u = ignition::math::Rand::DblUniform(0,1)+ignition::math::Rand::DblUniform(0,1);
    double r;
    if (u > 1){
        r = 2 -u;
    } else {
        r = u;
    }
    return ignition::math::Vector3d(radius*r*std::cos(t), radius*r*std::sin(t), 0);
}


//returns area of maximum rectangle that can fit in the histogram hist. l stores the leftmost index of the rectangle and r stores 1+(rightmost index)
int MaxHistogramArea(std::vector<int> hist, int &l, int &r){
    std::vector<int> stack; //make this a vector of (bar height, bar index)

    int max_area = 0;
    int tp;
    int area_with_top;
    int i =0;

    while (i<hist.size()){
        if (stack.empty() || hist[stack.back()] <= hist[i]){
            stack.push_back(i++);
        }else{
            tp = stack.back();
            stack.pop_back();

            area_with_top = hist[tp]*(stack.empty() ? i:(i-stack.back()-1));

            if (max_area < area_with_top){
                max_area = area_with_top;
                l = (stack.empty() ? 0:(stack.back()+1));
                r = i;
            }

        }
    }

    while (stack.empty() == false) { 
        tp = stack.back(); 
        stack.pop_back(); 
      
        area_with_top = hist[tp]*(stack.empty() ? i:(i-stack.back()-1));
  
        if (max_area < area_with_top){
            max_area = area_with_top; 
            
            l = (stack.empty() ? 0:(stack.back()+1));
            r = i;
            
        } 
            
    } 
    
   
    return max_area; 
}



// returns the maxiumum area rectangle of 1's in binary. Tuple.r = (min_r,min_c) Tuple.c = (max_r, max_c)[inclusive]
Tuple<Tuple<int>> MaxRectangle(std::vector<std::vector<int>> binary){
    std::vector<int> hist(binary[0].size(), 0);
    int max_area = 0;
    int left;
    int right;
    int max_row;

    for (int r =0; r<binary.size(); r++){
        for (int c = 0; c<hist.size(); c++){
            hist[c]=((binary[r][c] == 0) ? 0 : hist[c]+binary[r][c]);
        }
        int le,ri;
        int area =  MaxHistogramArea(hist, le, ri);
        if(max_area < area){
            max_area = area;
            max_row =r;
            left = le;
            right = ri;
        }

    }

    int h = max_area/(right-left);

    Tuple<int> min = Tuple<int>(max_row-h+1, left);
    Tuple<int> max = Tuple<int>(max_row, right-1);

    Tuple<Tuple<int>> res = Tuple<Tuple<int>>(min,max);
    return res;
}

}