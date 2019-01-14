#include <stdio.h>
#include <stdlib.h>

#include "verify_two_view_matches.h"
#include "feature_correspondence.h"
#include "camera_intrinsics_prior.h"
#include "estimate_twoview_info.h"
#include "twoview_info.h"


#include <glog/logging.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
// #include <io.h>
// #include <direct.h>
#include <vector>
#include <assert.h>
#include <limits.h>
#include <algorithm>  
#include <omp.h>
using namespace std;


typedef struct Coord{
	int fid;
	double x;
	double y;
}Coord1;


void split(vector<string> &toks, const string &s, const string &delims)
{
	toks.clear();

	string::const_iterator segment_begin = s.begin();
	string::const_iterator current = s.begin();
	string::const_iterator string_end = s.end();

	while (true)
	{
		if (current == string_end || delims.find(*current) != string::npos || *current == '\r')
		{
			if (segment_begin != current)
				toks.push_back(string(segment_begin, current));

			if (current == string_end || *current == '\r')
				break;

			segment_begin = current + 1;
		}

		current++;
	}

}

bool VerifyTwoViewMatches(
	const VerifyTwoViewMatchesOptions& options,
	const CameraIntrinsicsPrior& intrinsics1,
	const CameraIntrinsicsPrior& intrinsics2,
	const std::vector<FeatureCorrespondence>& correspondences,
	TwoViewInfo* twoview_info,
	std::vector<int>* inlier_indices) {
	if (correspondences.size() < options.min_num_inlier_matches) {
		return false;
	}

	// Estimate the two view info. If we fail to estimate a two view info then do
	// not add this view pair to the verified matches.
	if (!EstimateTwoViewInfo(options.estimate_twoview_info_options,
		intrinsics1,
		intrinsics2,
		correspondences,
		twoview_info,
		inlier_indices)) {
		return false;
	}

	// If there were not enough inliers, return false and do not bother to
	// (potentially) run bundle adjustment.
	if (inlier_indices->size() < options.min_num_inlier_matches) {
		return false;
	}

	// Bundle adjustment (optional).
	/*if (options.bundle_adjustment) {
	std::vector<FeatureCorrespondence> inliers(inlier_indices->size());
	for (int i = 0; i < inliers.size(); i++) {
	inliers[i] = correspondences[inlier_indices->at(i)];
	}
	if (!BundleAdjustRelativePose(inliers,
	intrinsics1,
	intrinsics2,
	twoview_info)) {
	return false;
	}
	}*/
	return true;
}


int main(int argc, char * argv[])
{
	ifstream myfile, myfile2;
	vector<string> names;
	string line, s2;;
	if (argc <4)
	{
		printf("Format: exec <path> <focalfile> <matchesfile> ");
		exit(1);
	}
	string motherpath = argv[1];
	string focalfile = motherpath + argv[2];
	string pointfile = motherpath + argv[3];
	string pointfileout = motherpath + "matches_forRtinlier5point.txt";
	string Rfile = motherpath + "R5point.txt";
	string Efile = motherpath + "E5point.txt";
	string Tfile = motherpath + "T5point.txt";
	string pairfile = motherpath + "original_pairs5point.txt";
	myfile2.open(focalfile);
	vector < double> camerafocal;
	int numcameras = 0;
	while (getline(myfile2, s2)) {
		vector <string> toks1;
		split(toks1, s2, " ");
		if (toks1.size() == 2)
		{
			camerafocal.push_back(stod(toks1[1]));
		}
		else
		{
			camerafocal.push_back(stod(toks1[2]));
		}

		numcameras++;
	}
	myfile2.close();

	//	printf("focal read = %lf\n",camerafocal[5]);


	FILE *fp = fopen(pointfileout.c_str(), "w");
	FILE *fpR = fopen(Rfile.c_str(), "w");
	FILE *fpE = fopen(Efile.c_str(), "w");
	FILE *fpT = fopen(Tfile.c_str(), "w");
	FILE *fpPairs = fopen(pairfile.c_str(), "w");
	fprintf(fp, "                                  \n");
	int numoutmatches = 0;

	myfile2.open(pointfile);
	getline(myfile2, line);
	int numTotalMatches = stoi(line);
	//printf("numTotalMatches = %d\n",numTotalMatches);
	VerifyTwoViewMatchesOptions options;
	
	options.bundle_adjustment = false;
	options.min_num_inlier_matches = 10;
	options.estimate_twoview_info_options.max_sampson_error_pixels = 2.25;


	for (int i = 0; i < numTotalMatches; i++)
	{
		vector <string> toks;
		getline(myfile2, line);
		split(toks, line, " ");
		int id1 = stoi(toks[0]);
		int id2 = stoi(toks[1]);
		int numMatches = stoi(toks[2]);
		vector <pair<Coord1, Coord1>> data;
		//v2_t *k1_pts = new v2_t[numMatches];
		//v2_t *k2_pts = new v2_t[numMatches];
		//k1_pts.resize(numMatches);
		//k2_pts.resize(numMatches);
		data.resize(numMatches);
		//		printf("numMatches read = %d\n",numMatches);
		
		
		
		CameraIntrinsicsPrior intrinsics1, intrinsics2;
		
		//setting K for image1
		intrinsics1.focal_length.value = camerafocal[id1];
		intrinsics1.focal_length.is_set = true;
		
		intrinsics1.principal_point[0].is_set = true;
		intrinsics1.principal_point[0].value = 0.0;
		intrinsics1.principal_point[1].is_set = true;
		intrinsics1.principal_point[1].value = 0.0;

		intrinsics1.aspect_ratio.is_set = true;
		intrinsics1.aspect_ratio.value = 1.0;

		intrinsics1.skew.is_set = true;
		intrinsics1.skew.value = 0.0;


		//setting K for image2
		intrinsics2.focal_length.value = camerafocal[id2];
		intrinsics2.focal_length.is_set = true;

		intrinsics2.principal_point[0].is_set = true;
		intrinsics2.principal_point[0].value = 0.0;
		intrinsics2.principal_point[1].is_set = true;
		intrinsics2.principal_point[1].value = 0.0;

		intrinsics2.aspect_ratio.is_set = true;
		intrinsics2.aspect_ratio.value = 1.0;

		intrinsics2.skew.is_set = true;
		intrinsics2.skew.value = 0.0;


		
		
		std::vector<FeatureCorrespondence> correspondences;
		for (int j = 0; j < numMatches; j++)
		{
			vector <string> toks2;
			string line2;
			getline(myfile2, line2);
			split(toks2, line2, " ");

			FeatureCorrespondence tmp;
			tmp.feature1.x() = stod(toks2[1]);
			tmp.feature1.y() = stod(toks2[2]);
			tmp.feature2.x() = stod(toks2[4]);
			tmp.feature2.y() = stod(toks2[5]);
			correspondences.push_back(tmp);
			data[j].first.fid = stoi(toks2[0]);
			data[j].first.x = stod(toks2[1]);
			data[j].first.y = stod(toks2[2]);
			data[j].second.fid = stoi(toks2[3]);
			data[j].second.x = stod(toks2[4]);
			data[j].second.y = stod(toks2[5]);
			/*k1_pts[j] = v2_new(data[j].first.x, data[j].first.y);
			k2_pts[j] = v2_new(data[j].second.x, data[j].second.y);*/
			//	printf("%d %lf %lf %d %lf %lf\n",data[j].first.fid,data[j].first.x, data[j].first.y,data[j].second.fid,data[j].second.x, data[j].second.y);

		}

		TwoViewInfo twoview_info;
		std::vector<int> inlier_indices;
		bool ret = VerifyTwoViewMatches(options, intrinsics1, intrinsics2, correspondences, &twoview_info, &inlier_indices);
		if (ret)
		{
			int num_inliers = inlier_indices.size();
			fprintf(fp, "%d %d %d\n", id1, id2, num_inliers);
			for (int j = 0; j<num_inliers; j++)
			{
				int loc = inlier_indices[j];
				fprintf(fp, "%d %f %f %d %f %f\n", data[loc].first.fid, data[loc].first.x, data[loc].first.y, data[loc].second.fid, data[loc].second.x, data[loc].second.y);
				
			}
			fprintf(fpR, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", twoview_info.rotationmat_2(0, 0), twoview_info.rotationmat_2(0, 1), twoview_info.rotationmat_2(0, 2), twoview_info.rotationmat_2(1, 0), twoview_info.rotationmat_2(1, 1), twoview_info.rotationmat_2(1, 2), twoview_info.rotationmat_2(2, 0), twoview_info.rotationmat_2(2, 1), twoview_info.rotationmat_2(2, 2));
			fprintf(fpE, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", twoview_info.essential_mat(0, 0), twoview_info.essential_mat(0, 1), twoview_info.essential_mat(0, 2), twoview_info.essential_mat(1, 0), twoview_info.essential_mat(1, 1), twoview_info.essential_mat(1, 2), twoview_info.essential_mat(2, 0), twoview_info.essential_mat(2, 1), twoview_info.essential_mat(2, 2));
			fprintf(fpT, "%lf %lf %lf\n", twoview_info.translation_2(0), twoview_info.translation_2(1), twoview_info.translation_2(2));
			fprintf(fpPairs, "%d %d %lf %lf\n", id1 + 1, id2 + 1, twoview_info.cost, 1.0);
			numoutmatches++;
		}
		correspondences.clear();
		data.clear();
		/*if (numMatches > 10)
		{
		}*/
	}
	fseek(fp, 0, SEEK_SET);
	fprintf(fp, "%d", numoutmatches);
	fclose(fp);
	fclose(fpR);
	fclose(fpE);
	fclose(fpT);
	fclose(fpPairs);
	myfile2.close();
}