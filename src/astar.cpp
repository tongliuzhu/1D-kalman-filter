#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#define PIXEL_SIZE 40

using namespace std;
using namespace cv;
Mat img(600, 1000, CV_8UC3, Scalar(255, 255, 255));

struct CPoint
{
	CPoint(int x, int y) : X(x), Y(y), G(0), H(0), F(0), m_parentPoint(NULL){};
	~CPoint();

	void CalF()
	{
		F = G + H;
	};
	int X, Y, G, H, F;
	CPoint *m_parentPoint;
};
class AStar
{
private:
	vector<vector<int>> m_array;

	typedef std::vector<CPoint *> POINTVEC;
	POINTVEC m_openVec;
	POINTVEC m_closeVec;

public:
	AStar(vector<vector<int>>& array)
	{
		m_array.resize(array.size());
		for (int i = 0; i < array.size(); i++)
		{
			m_array[i].resize(array[i].size());
			for(int j = 0; j < array[0].size(); j++)
			{
				m_array[i][j] = array[i][j];
			}
		}		
	}
	CPoint *GetMinFPoint()
	{
		int idx = 0, valueF = 9999;
		for (int i = 0; i < m_openVec.size(); ++i)
		{
			if (m_openVec[i]->F < valueF)
			{
				valueF = m_openVec[i]->F;
				idx = i;
			}
		}
		return m_openVec[idx];
	}

	bool RemoveFromOpenVec(CPoint *point)
	{
		for (POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); ++it)
		{
			if ((*it)->X == point->X && (*it)->Y == point->Y)
			{
				m_openVec.erase(it);
				return true;
			}
		}
		return false;
	}

	bool canReach(int x, int y)
	{
		return 0 == m_array[x][y];
	}

	bool IsAccessiblePoint(CPoint *point, int x, int y, bool canDiagDirection)
	{
		if (!canReach(x, y) || isInCloseVec(x, y))
			return false;
		else
		{
			//可到达的点
			if (abs(x - point->X) + abs(y - point->Y) == 1) // 左右上下点
				return true;
			else
			{
				if (canDiagDirection && abs(x - point->X) + abs(y - point->Y) == 2) // 对角点
					return true;
				else
					return false; //不能斜着走
			}
		}
	}

	std::vector<CPoint *> GetAdjacentPoints(CPoint *point, bool canDiagDirection)
	{
		POINTVEC adjacentPoints;
		for (int x = point->X - 1; x <= point->X + 1; ++x)
			for (int y = point->Y - 1; y <= point->Y + 1; ++y)
			{
				if(x < 0 || x >= m_array.size() || y < 0 || y >= m_array[0].size())
				{
					continue; // if outside of the map, do nothing // if not core dump
				}
				if (IsAccessiblePoint(point, x, y, canDiagDirection))
				{
					CPoint *tmpPoint = new CPoint(x, y);
					adjacentPoints.push_back(tmpPoint);
				}
			}

		return adjacentPoints;
	}

	bool isInOpenVec(int x, int y)
	{
		for (POINTVEC::iterator it = m_openVec.begin(); it != m_openVec.end(); ++it)
		{
			if ((*it)->X == x && (*it)->Y == y)
				return true;
		}
		return false;
	}

	bool isInCloseVec(int x, int y)
	{
		for (POINTVEC::iterator it = m_closeVec.begin(); it != m_closeVec.end(); ++it)
		{
			if ((*it)->X == x && (*it)->Y == y)
				return true;
		}
		return false;
	}

	void RefreshPoint(CPoint *tmpStart, CPoint *point)
	{
		int valueG = CalcG(tmpStart, point);
		if (valueG < point->G)
		{
			point->m_parentPoint = tmpStart;
			point->G = valueG;
			point->CalF();
		}
	}

	void AddInOpenVec(CPoint *tmpStart, CPoint *end, CPoint *point)
	{
		point->m_parentPoint = tmpStart;
		point->G = CalcG(tmpStart, point);
		point->G = CalcH(end, point);
		point->CalF();
		m_openVec.push_back(point);
	}

	int CalcG(CPoint *start, CPoint *point)
	{
		int Gx = abs(point->X - start->X);
		int Gy = abs(point->Y - start->Y);
		int G = pow(Gx, 2) + pow(Gy, 2); // 利用欧氏距离不开根，计算G和F
		int parentG = point->m_parentPoint != NULL ? point->m_parentPoint->G : 0;
		return G + parentG;
	}

	int CalcH(CPoint *end, CPoint *point)
	{
		int Fx = abs(point->X - end->X);
		int Fy = abs(point->Y - end->Y);
		int F = pow(Fx, 2) + pow(Fy, 2);
		return F;
	}

	// 搜索路径
	CPoint *FindPath(CPoint *start, CPoint *end, bool canDiagDirection)
	{
		m_openVec.push_back(start);
		while (0 != m_openVec.size())
		{
			CPoint *tmpStart = GetMinFPoint(); // 获取F值最小的点
			RemoveFromOpenVec(tmpStart);
			m_closeVec.push_back(tmpStart);

			Point left_up, right_bottom;
			left_up.x = tmpStart->Y * PIXEL_SIZE; //存储数组的列(point->Y)对应矩形的x轴，一个格大PIXEL_SIZE像素
			left_up.y = tmpStart->X * PIXEL_SIZE;
			right_bottom.x = left_up.x + PIXEL_SIZE;
			right_bottom.y = left_up.y + PIXEL_SIZE;
			rectangle(img, left_up, right_bottom, Scalar(255, 0, 0), CV_FILLED, 8, 0); //path yellow(full)

			POINTVEC adjacentPoints = GetAdjacentPoints(tmpStart, canDiagDirection);
			for (POINTVEC::iterator it = adjacentPoints.begin(); it != adjacentPoints.end(); ++it)
			{
				CPoint *point = *it;
				if (isInOpenVec(point->X, point->Y)) // 在开启列表
					RefreshPoint(tmpStart, point);
				else
				{
					Point left_up, right_bottom;
					left_up.x = point->Y * PIXEL_SIZE; //存储数组的列(point->Y)对应矩形的x轴，一个格大PIXEL_SIZE像素
					left_up.y = point->X * PIXEL_SIZE;
					right_bottom.x = left_up.x + PIXEL_SIZE;
					right_bottom.y = left_up.y + PIXEL_SIZE;
					rectangle(img, left_up, right_bottom, Scalar(0, 0, 255), CV_FILLED, 8, 0); //new add openlist(edge)
					AddInOpenVec(tmpStart, end, point);
				}
			}
			imshow("Test", img); //窗口中显示图像
			waitKey(1000); // display the image for 1000ms
			if (isInOpenVec(end->X, end->Y)) // 目标点已经在开启列表中
			{
				for (int i = 0; i < m_openVec.size(); ++i)
				{
					if (end->X == m_openVec[i]->X && end->Y == m_openVec[i]->Y)
						return m_openVec[i];
				}
			}
		}
		return end;
	}
};

int main()
{
	int start_point_x = 1;
	int start_point_y = 1;
	int goal_point_x = 12;
	int goal_point_y = 16;
	vector<vector<int>> array =
		{
		   //0  1  2  3  4  5  6  7  8  9 10  11,12,13,14,15,16,17,18,19,20,21,22,23,24
			{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // 0
			{1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1}, // 1
			{1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1}, // 2
			{1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1}, // 3
			{1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1}, // 4
			{1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1}, // 5
			{1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1}, // 6
			{1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1}, // 7
			{1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1}, // 8
			{1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1}, // 9
			{1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1}, // 10
			{1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1}, // 11
			{1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1}  // 12
		};

	AStar *pAStar = new AStar(array);
	if (array[start_point_x][start_point_y] || array[goal_point_x][goal_point_y])
	{
		cout << "start point or goal point set error!!!" << endl;
		return 0;
	}
	namedWindow("Test");

	Rect rect;
	Point left_up, right_bottom;
	for (int i = 0; i < array.size(); i++)
	{
		for (int j = 0; j < array[0].size(); j++)
		{
			left_up.x = j * PIXEL_SIZE; //存储数组的列(j)对应矩形的x轴
			left_up.y = i * PIXEL_SIZE;
			right_bottom.x = left_up.x + PIXEL_SIZE;
			right_bottom.y = left_up.y + PIXEL_SIZE;
			if (array[i][j])
			{
				rectangle(img, left_up, right_bottom, Scalar(0, 0, 0), CV_FILLED, 8, 0); //obstacles 青色
			}
			else
			{
				if (i == start_point_x && j == start_point_y)
					rectangle(img, left_up, right_bottom, Scalar(255, 0, 0), CV_FILLED, 8, 0); //start point blue(full)
				else if (i == goal_point_x && j == goal_point_y)
					rectangle(img, left_up, right_bottom, Scalar(255, 0, 255), CV_FILLED, 8, 0); //goal point black(full)
				else
					rectangle(img, left_up, right_bottom, Scalar(0, 255, 0), 2, 8, 0); //free white content,green edge
			}
		}
	}

	imshow("Test", img); //窗口中显示图像
	CPoint *start = new CPoint(start_point_x, start_point_y);
	CPoint *end = new CPoint(goal_point_x, goal_point_y);
	CPoint *point = pAStar->FindPath(start, end, false);
	std::cout << __LINE__ << std::endl;

	while (point != NULL)
	{
		left_up.x = point->Y * PIXEL_SIZE; //存储数组的列(point->Y)对应矩形的x轴，一个格大PIXEL_SIZE像素
		left_up.y = point->X * PIXEL_SIZE;
		right_bottom.x = left_up.x + PIXEL_SIZE;
		right_bottom.y = left_up.y + PIXEL_SIZE;
		rectangle(img, left_up, right_bottom, Scalar(0, 255, 255), CV_FILLED, 8, 0); //path yellow(full)
		std::cout << "(" << point->X << "," << point->Y << ");" << std::endl;
		point = point->m_parentPoint;
	}
	imshow("Test", img); //窗口中显示图像
	waitKey(0);

	return 0;
}