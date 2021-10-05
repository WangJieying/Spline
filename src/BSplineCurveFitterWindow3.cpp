// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2020
// Distributed under the Boost Software License, Version 1.0.
// https://www.boost.org/LICENSE_1_0.txt
// https://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Version: 4.0.2019.08.13

#include "BSplineCurveFitterWindow3.h"
#include <Graphics/VertexColorEffect.h>
#include <random>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
  
 
using namespace std;
#define mDimension 3
 
int TotalControlNum; 
int DeterminedNumControls, DeterminedDegree;
float minError;
float minErrorThreshold;
float storevector[mDimension]={0,0,0};
vector<vector<vector<Vector3<float>>>> BSplineCurveFitterWindow3::IndexingCP = {{}};
vector<vector<vector<Vector3<float>>>> BSplineCurveFitterWindow3::IndexingCP_Interactive = {{}};

vector<Vector3<float>> ReadingSampleforEachCC = {{}};
vector<Vector3<float>> ReadingSampleforEachInty = {{}};
vector<vector<Vector3<float>>> CPforEachLayer_or_CC = {{}};

unique_ptr<BSplineCurveGenerate<float>> SplineGeneratePtr;
bool mergeOrNot = false;   
bool deleteshort = false;
unsigned int MinAllowableLength = 5;
float diagonal= 0.0f;

BSplineCurveFitterWindow3::BSplineCurveFitterWindow3()
{
    //cout<<"BSplineCurveFitterWindow-----"<<endl;
}
int BSplineCurveFitterWindow3::SplineFit(vector<vector<Vector3<float>>> BranchSet, float hausdorff_,float diagonal_, int layerNum, vector<int *> connection_)
{
    TotalControlNum = 0;
    if(!sampleSet.empty()) sampleSet.clear();
    sampleSet = BranchSet;
    minErrorThreshold = hausdorff_;
    diagonal = diagonal_;
    connection = connection_;
    
    if (mergeOrNot) {Merge();}

    for (unsigned int i = 0; i < sampleSet.size();i++)
    {
        if (sampleSet[i].size()>3) 
            CalculateNeededCP(sampleSet[i]);
    }
     
    return TotalControlNum; 
}

void BSplineCurveFitterWindow3::SplineFit2(vector<vector<Vector3<float>>> BranchSet, float hausdorff_,float diagonal_, int layerNum, vector<int *> connection_, int sign)
{
    if(!sampleSet.empty()) sampleSet.clear();
    sampleSet = BranchSet;
    minErrorThreshold = hausdorff_;
    diagonal = diagonal_;
    connection = connection_; 
    
    if (mergeOrNot) { Merge();}
    if(!CPforEachLayer_or_CC.empty()) CPforEachLayer_or_CC.clear();

    for (unsigned int i = 0; i < sampleSet.size();i++)
    {
        if (sampleSet[i].size()>3) {
            CreateBSplinePolyline(sampleSet[i]);
        }
    }
    IndexingCP_Interactive.push_back(CPforEachLayer_or_CC);
    //CPforEachLayer_or_CC.clear();
    
}

void BSplineCurveFitterWindow3::indexingSpline(vector<vector<Vector3<float>>> BranchSet, float hausdorff_,float diagonal_, int layerNum, int index)
{
    
    if(!sampleSet.empty()) sampleSet.clear();
    sampleSet = BranchSet;
    minErrorThreshold = hausdorff_;
    diagonal = diagonal_;
    
    if(!CPforEachLayer_or_CC.empty()) CPforEachLayer_or_CC.clear();

    for (unsigned int i = 0; i < sampleSet.size();i++)
    {
        if (sampleSet[i].size()>3){
            CreateBSplinePolyline(sampleSet[i]);
        } 
            
    }
    IndexingCP.push_back(CPforEachLayer_or_CC);
    
}

vector<vector<Vector3<float>>> BSplineCurveFitterWindow3::SplineGenerate()
{
    int CPnum,degree;
    unsigned int numSamples;
    vector<float> mControlData;

    vector<vector<Vector3<float>>> ReadingCPforEachInty, ReadingSampleforAllInty;
    vector<Vector3<float>> ReadingCPforEachBranch;
    Vector3<float> ReadingEachCP;
     
    for(auto it = IndexingCP_Interactive.begin();it!=IndexingCP_Interactive.end();it++){
        ReadingCPforEachInty = *it;
        //cout<<"ReadingCPforEachInty size "<<ReadingCPforEachInty.size()<<endl;
        for(auto it_ = ReadingCPforEachInty.begin();it_!=ReadingCPforEachInty.end();it_++){
            ReadingCPforEachBranch = *it_;
            bool first = true;
            for(auto it_branch = ReadingCPforEachBranch.begin(); it_branch != ReadingCPforEachBranch.end(); it_branch++){
                ReadingEachCP = *it_branch;
                if(first){
                    first = false;
                    CPnum = ReadingEachCP[0];
                    degree = ReadingEachCP[1];
                    numSamples = ReadingEachCP[2];
                }
                else{
                    for (int j = 0; j < mDimension; ++j)
                    {
                        mControlData.push_back(ReadingEachCP[j]/diagonal);
                    }
                }
            }
            //cout<<endl;
            SplineGeneratePtr = std::make_unique<BSplineCurveGenerate<float>>(mDimension, degree, mControlData, CPnum);
            
            CreateGraphics(numSamples, 0);
            mControlData.clear(); 

        }
        ReadingSampleforAllInty.push_back(ReadingSampleforEachInty);
        ReadingSampleforEachInty.clear();
    }
    return ReadingSampleforAllInty;   
}


vector<Vector3<float>> BSplineCurveFitterWindow3::ReadIndexingSpline(vector<vector<Vector3<float>>> cpList)
{
    int CPnum,degree;
    unsigned int numSamples;
    vector<float> mControlData;

    vector<Vector3<float>> ReadingCPforEachBranch;
    Vector3<float> ReadingEachCP;
    if(!ReadingSampleforEachCC.empty()) ReadingSampleforEachCC.clear();
    
    for(auto it_ = cpList.begin();it_!=cpList.end();it_++){
        ReadingCPforEachBranch = *it_;
        bool first = true;
        for(auto it_branch = ReadingCPforEachBranch.begin(); it_branch != ReadingCPforEachBranch.end(); it_branch++){
            ReadingEachCP = *it_branch;
            if(first){
                first = false;
                CPnum = ReadingEachCP[0];
                degree = ReadingEachCP[1];
                numSamples = ReadingEachCP[2];
            }
            else{
                for (int j = 0; j < mDimension; ++j)
                {
                    mControlData.push_back(ReadingEachCP[j]/diagonal);
                }

            }
        }
        SplineGeneratePtr = std::make_unique<BSplineCurveGenerate<float>>(mDimension, degree, mControlData, CPnum);
        
        CreateGraphics(numSamples, 1);
        mControlData.clear(); 

    }
        
    return ReadingSampleforEachCC;   
}

vector<vector<Vector3<float>>> BSplineCurveFitterWindow3::ReadIndexingSpline()
{
    int CPnum,degree;
    unsigned int numSamples;
    //OutFile.open("sample.txt");
    vector<float> mControlData;

    vector<vector<Vector3<float>>> ReadingCPforEachCC, ReadingSampleforAllCC;
    vector<Vector3<float>> ReadingCPforEachBranch;
    Vector3<float> ReadingEachCP;
    if(!ReadingSampleforEachCC.empty()) ReadingSampleforEachCC.clear();
    
    
    for(auto it = IndexingCP.begin();it!=IndexingCP.end();it++){
        ReadingCPforEachCC = *it;
        for(auto it_ = ReadingCPforEachCC.begin();it_!=ReadingCPforEachCC.end();it_++){
            ReadingCPforEachBranch = *it_;
            bool first = true;
            for(auto it_branch = ReadingCPforEachBranch.begin(); it_branch != ReadingCPforEachBranch.end(); it_branch++){
                ReadingEachCP = *it_branch;
                if(first){
                    first = false;
                    CPnum = ReadingEachCP[0];
                    degree = ReadingEachCP[1];
                    numSamples = ReadingEachCP[2];
                }
                else{
                    for (int j = 0; j < mDimension; ++j)
                    {
                        mControlData.push_back(ReadingEachCP[j]/diagonal);
                    }

                }
            }
            SplineGeneratePtr = std::make_unique<BSplineCurveGenerate<float>>(mDimension, degree, mControlData, CPnum);
           
            CreateGraphics(numSamples, 1);
            mControlData.clear(); 

        }
        //OutFile << 65535 <<endl;
        ReadingSampleforAllCC.push_back(ReadingSampleforEachCC);
        ReadingSampleforEachCC.clear();
    }
    return ReadingSampleforAllCC;        
    
}


void BSplineCurveFitterWindow3::CreateGraphics(unsigned int numSamples, int which)
{
    
    unsigned int numSplineSample = (unsigned int)(numSamples*1.1);//sub-pixel.
   // unsigned int numSplineSample = numSamples; //uniform sampling
    float vector[mDimension]={0,0,0};
    float multiplier = 1.0f / (numSplineSample - 1.0f);
    Vector3<float> EachSample;

    for (unsigned int i = 0; i < numSplineSample; ++i)
    { 
        float t = multiplier * i;
       
        SplineGeneratePtr->GetPosition(t, reinterpret_cast<float*>(vector));
        //OutFile<<(int)(vector[0]*diagonal)<<" "<<(int)(vector[1]*diagonal)<<" "<<(int)(vector[2]*diagonal)<<endl;      //save to the txt file.
        
        for (int j = 0; j < mDimension; ++j)
            EachSample[j] = (int)(vector[j]*diagonal);
        
        if(which) ReadingSampleforEachCC.push_back(EachSample);
        else ReadingSampleforEachInty.push_back(EachSample);
        
    }
}

float BSplineCurveFitterWindow3::Judge(vector<Vector3<float>> Sample)
{
    unsigned int numSamples = (unsigned int)Sample.size();
    vector<Vector3<float>> SplineSamples(10000);
    
    float CPandError = 0;
    
    unsigned int numSplineSamples = (unsigned int)(numSamples * 1.4);
    //unsigned int numSplineSamples = numSamples; // uniform sampling.
    float multiplier = 1.0f / (numSplineSamples - 1.0f);
    SplineSamples.resize(numSplineSamples);
    
    int minDegree;
    int numControls = 2;
   // int iter = 0;
    while (1)
    {
        //iter ++;
        minError = 100.0f; minDegree = 10;
        for (int degree = 1; degree < numControls; degree++)
        {
            if (degree > 10) break;
            mSpline = std::make_unique<BSplineCurveFit<float>>(mDimension, static_cast<int>(Sample.size()),
            reinterpret_cast<float const*>(&Sample[0]), degree, numControls);

            for (unsigned int i = 0; i < numSplineSamples; ++i)
            {
                float t = multiplier * i;
                mSpline->GetPosition(t, reinterpret_cast<float*>(storevector));
                 for(int y=0;y<mDimension;y++)
                    SplineSamples[i][y] = storevector[y];
            }
            
        // Compute error measurements.
            float maxLength = 0.0f;
            Vector3<float> diff;
            float sqrLength, minLength;
            for (unsigned int i = 0; i < numSamples; ++i)
            {
                minLength = 100.0f;
                for (unsigned int j = 0; j < numSplineSamples; ++j)
                {
                    diff = Sample[i] - SplineSamples[j];
                    sqrLength = Dot(diff, diff);
                    if (sqrLength < minLength) {minLength = sqrLength; if (minLength == 0) break;}
                }
                if (minLength > maxLength) maxLength = minLength;
            
            }
            hausdorff = std::sqrt(maxLength);
            if (minError > hausdorff) { minError = hausdorff; minDegree = degree;}
            //cout<<numControls<<" hausdorff: "<<hausdorff<<" degree: "<<degree<<endl;
        }
        //cout<<numControls<<" minError: "<<minError<<" minDegree: "<<minDegree<<endl;
        if (numControls > 12) //If numControl > 13, it's easy to get crush.
        {
            CPandError = 100;//assign an big enough value
            return CPandError;
        }
        if (minError < minErrorThreshold)
        {
            DeterminedNumControls = numControls;
            DeterminedDegree = minDegree;
            //cout<<" minError: "<<minError<<endl;
            CPandError = numControls + minError;

            break;
        }
        else numControls ++;
    }
   
    return CPandError;
}

void BSplineCurveFitterWindow3::CalculateNeededCP(vector<Vector3<float>> Sample)
{
    float cpError = Judge(Sample);
    
    if (cpError == (float)100) //the branch may be too long to fit well.
    {
        vector<Vector3<float>> first = Sample;//init to VOID segmentation fault.
        vector<Vector3<float>> second = Sample;
        
        for (unsigned int i = Sample.size()/2; i < Sample.size(); ++i) //split in the half
            first[i] = 0;
        first.resize(Sample.size()/2);  
        for (unsigned int i = Sample.size()/2; i < Sample.size(); ++i)
            second[i - Sample.size()/2] = Sample[i];
        second.resize(Sample.size() - Sample.size()/2);  
   
        CalculateNeededCP(first);
        CalculateNeededCP(second);
    }
    else 
        TotalControlNum += DeterminedNumControls;
}

void BSplineCurveFitterWindow3::CreateBSplinePolyline(vector<Vector3<float>> Sample)
{
    float cpError = Judge(Sample);
    
    if (cpError == (float)100) //the branch may be too long to fit well.
    {
        cout<<"---------"<<endl;
        vector<Vector3<float>> first = Sample;//init to VOID segmentation fault.
        vector<Vector3<float>> second = Sample;
        
        for (unsigned int i = Sample.size()/2; i < Sample.size(); ++i) //split in the half
            first[i] = 0;
        first.resize(Sample.size()/2);  
        for (unsigned int i = Sample.size()/2; i < Sample.size(); ++i)
            second[i - Sample.size()/2] = Sample[i];
        second.resize(Sample.size() - Sample.size()/2);  
   
        CreateBSplinePolyline(first);
        CreateBSplinePolyline(second);
    }
    else
    {                      
        mSpline = std::make_unique<BSplineCurveFit<float>>(mDimension, static_cast<int>(Sample.size()),
            reinterpret_cast<float const*>(&Sample[0]), DeterminedDegree, DeterminedNumControls);
    
        //cout<< DeterminedNumControls <<" "<<DeterminedDegree<<" "; 
        //cout<< Sample.size() <<" "; 
        vector<Vector3<float>> CPforEachBranch; 
        Vector3<float> eachTriple;
        eachTriple[0] = DeterminedNumControls;
        eachTriple[1] = DeterminedDegree;
        eachTriple[2] = Sample.size();
        CPforEachBranch.push_back(eachTriple);
        
        
        float const* controlDataPtr = mSpline->GetControlData();
        for (int i = 0; i< DeterminedNumControls; ++i)
        {
            for (int j = 0; j < mDimension; ++j)
            {
                //controlData[i][j] = (*controlDataPtr);
                //cout<<(round)(*controlDataPtr*diagonal)<<" ";
                eachTriple[j] = (round)(*controlDataPtr*diagonal);
                controlDataPtr++;
            }
            CPforEachBranch.push_back(eachTriple);
        
        }    
        //cout<<"CPforEachLayer_or_CC.size "<<CPforEachLayer_or_CC.size()<<endl;
        CPforEachLayer_or_CC.push_back(CPforEachBranch);
    }
}

void BSplineCurveFitterWindow3::Merge()
{
    vector<Vector3<float>> first, second;
    float minEandContlNum = 100.0;
    //float maxdiff = 0.0;
    int minIndex = 1000;
    float firstE,secondE,mergeE;
    
    //vector<int> GapFill;////
    //outMerge.open("outMerge.txt");
  //time:0.007
    //clock_t start = clock();
    for(auto it = connection.begin();it!=connection.end();it++)
    {
        int *sampleIndex = *it;
        //cout<<out[0]<<"--"<<out[1]<<"--"<<out[2]<<"--"<<out[3]<<endl;
        first = sampleSet[sampleIndex[0]];
        if (deleteshort) firstE = Judge(first);
        else 
        {
            if (first.size()<MinAllowableLength) firstE = 2.01;//
            else firstE = Judge(first);
        }

        for(int index = 1; index < 4; index++)//index for 'sampleIndex'.
        {
            if(sampleIndex[index]==0) continue;
            second = sampleSet[sampleIndex[index]];
                
            if (deleteshort) secondE = Judge(second);
            else 
            {
                if(second.size()<MinAllowableLength) secondE = 2.01;//assign a big num.
                else secondE = Judge(second);
            }

            merge.resize(10000);//This is very important!!
            for (unsigned int i = 0; i < first.size(); ++i)
                merge[i] = first[i];
            
            for (unsigned int i = first.size(); i < (first.size()+second.size()); ++i)
                merge[i] = second[i-first.size()];
                
            merge.resize(first.size()+second.size());
           

            if (deleteshort) mergeE = Judge(merge);
            else
            {
                if (merge.size()<MinAllowableLength) mergeE = 4; //about 3CP + 0.- error.
                else mergeE = Judge(merge);
            }
    
            //cout<<"mergeE: "<<mergeE<<" firstE: "<<firstE<<" secondE: "<<secondE<<endl;
            if (mergeE < (firstE + secondE)) 
                if (minEandContlNum > mergeE) { minEandContlNum = mergeE; minIndex = sampleIndex[index];}
            
        }
        if (minIndex!=1000) //meet the merge condition.
        {
            //cout<<"Merge！ first: "<<sampleIndex[0]<<" second: "<<minIndex<<endl;
            second = sampleSet[minIndex];
            merge.resize(10000);//This is very important!!
            for (unsigned int i = 0; i < first.size(); ++i)
                merge[i] = first[i];
            
            for (unsigned int i = first.size(); i < (first.size()+second.size()); ++i)
                merge[i] = second[i-first.size()];
            merge.resize(first.size()+second.size());

            //for (unsigned int i = 0; i<merge.size(); i++)
            //    outMerge<<iter<<" "<<merge[i][0]*diagonal<<" "<<merge[i][1]*diagonal<<" "<<merge[i][2]*diagonal<<endl;
        

            sampleSet.erase(sampleSet.begin()+sampleIndex[0]);
            sampleSet.insert(sampleSet.begin()+sampleIndex[0],merge);
            //sampleSet.erase(sampleSet.begin()+minIndex);
            sampleSet[minIndex].clear();//still occupy the position.

            minEandContlNum = 100.0f;
            minIndex = 1000;
        }
        
    }
    
    //cout<<"--"<<sampleSet.size()<<endl;
        for (unsigned int i = sampleSet.size()-1; i > 0;i--)
        {
            if (sampleSet[i].size()<1) 
                sampleSet.erase(sampleSet.begin()+i);//when merged two branches, we'll leave one branch empty, 
            //cout<<"--"<<i<<endl;                   //so we squeeze the vector, that' why we start from the end.

        }
        
    

    //outMerge.close();
    //cout<<"--"<<sampleSet.size()<<endl;
     
}
