using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace _2_convex_hull
{
    class ConvexHullSolver
    {
        System.Drawing.Graphics g;
        System.Windows.Forms.PictureBox pictureBoxView;

        public ConvexHullSolver(System.Drawing.Graphics g, System.Windows.Forms.PictureBox pictureBoxView)
        {
            this.g = g;
            this.pictureBoxView = pictureBoxView;
        }

        public void Refresh()
        {
            // Use this especially for debugging and whenever you want to see what you have drawn so far
            pictureBoxView.Refresh();
        }

        public void Pause(int milliseconds)
        {
            // Use this especially for debugging and to animate your algorithm slowly
            pictureBoxView.Refresh();
            System.Threading.Thread.Sleep(milliseconds);
        }

        public void Solve(List<System.Drawing.PointF> points)
        {
            //MAKE SURE THAT ALL POINTS ARE ORDERED BY X VALUE
            points = points.OrderBy(p => p.X).ToList(); // O(n) = nlog(n), assuming it uses stable quicksort (thanks google)
            //Begin recursive solving
            ConvexHull finalHull = RecursiveSolve(points);
            //Get all the points found from the recursive solving and draw them
            PointF[] array = finalHull.GetAllPoints().ToArray();
            g.DrawLines(new Pen(Color.Green, 3), array);
            g.DrawLine(new Pen(Color.Green, 3), array[0], array[finalHull.GetAllPoints().Count - 1]);
            Refresh();
        }

        // Recursively solves by splitting the problem into two smaller pieces with half as many items
        // T(n) = 2T(n/2) + O(n) =>  a=2 b=2 d=1 => 1= log2(2) => O(n) = nlog(n)
        public ConvexHull RecursiveSolve(List<PointF> pointList)
        {
            if (pointList.Count <= 1)
            {
                ConvexHull result = new ConvexHull(pointList);
                result.SetFarthestRight(pointList[pointList.Count - 1]);
                return result;
            }
            else
            {
                ConvexHull left = RecursiveSolve(pointList.Take(pointList.Count / 2).ToList());
                ConvexHull right = RecursiveSolve(pointList.Skip(pointList.Count / 2).ToList());
                return MergeMe(left, right);
            }
        }

        //Merges the outer hulls, skipping the inner hulls and adding the newly found hull lines
        public ConvexHull MergeMe(ConvexHull left, ConvexHull right)
        {
            KeyValuePair<int, int> holder = TopTangentFinder(left, right);
            int topLeft = holder.Key;
            int topRight = holder.Value;

            holder = BottomTangentFinder(left, right);
            int bottomLeft = holder.Key;
            int bottomRight = holder.Value;

            //Get all the boundry points from the current left and right ConvexHulls, making sure to catch the new boundary points
            List<PointF> boundaryPoints = new List<PointF>();
            for (int i = 0; i <= topLeft; i++)
            {
                boundaryPoints.Add(left.GetAllPoints()[i]);
            }
            for (int i = topRight; i != bottomRight; i = right.GetNextIndex(i))
            {
                boundaryPoints.Add(right.GetAllPoints()[i]);
            }
            boundaryPoints.Add(right.GetAllPoints()[bottomRight]);
            for (int i = bottomLeft; i != 0; i = left.GetNextIndex(i))
            {
                boundaryPoints.Add(left.GetAllPoints()[i]);
            }
            return new ConvexHull(boundaryPoints);
        }

        //Find the top tangent between two hulls. I wanted to combine this one with BottomTangentFinder, but the alternating requirements convoluted the logic
        private KeyValuePair<int, int> TopTangentFinder(ConvexHull left, ConvexHull right)
        {
            int currentLeftIndex = left.GetFurthestRightIndex();
            int currentRightIndex = right.GetFurthestLeftIndex();
            bool doOnce = true;
            bool leftChanged = false;
            bool rightChanged = false;
            int topLeft = -1;
            int topRight = -1;
            List<PointF> rightPoints = right.GetAllPoints();
            List<PointF> leftPoints = left.GetAllPoints();
            while (doOnce || leftChanged || rightChanged)
            {
                if (doOnce || leftChanged)
                {
                    topRight = currentRightIndex;
                    while (FindSlope(leftPoints[currentLeftIndex], rightPoints[right.GetNextIndex(topRight)]) >
                            FindSlope(leftPoints[currentLeftIndex], rightPoints[topRight]))
                    {
                        topRight = right.GetNextIndex(topRight);
                    }
                    if (topRight == currentRightIndex)
                    {
                        leftChanged = false;
                        rightChanged = false;
                    }
                    else
                    {
                        rightChanged = true;
                        currentRightIndex = topRight;
                    }
                }
                if (doOnce || rightChanged)
                {
                    topLeft = currentLeftIndex;
                    while (FindSlope(rightPoints[currentRightIndex], leftPoints[left.GetPreviousIndex(topLeft)]) <
                            FindSlope(rightPoints[currentRightIndex], leftPoints[topLeft]))
                    {
                        topLeft = left.GetPreviousIndex(topLeft);
                    }
                    if (topLeft == currentLeftIndex)
                    {
                        leftChanged = false;
                        rightChanged = false;
                    }
                    else
                    {
                        leftChanged = true;
                        currentLeftIndex = topLeft;
                    }
                }
                doOnce = false;
            }
            return new KeyValuePair<int, int>(topLeft, topRight);
        }

        //Find the bottom tangent between two hulls.
        private KeyValuePair<int, int> BottomTangentFinder(ConvexHull left, ConvexHull right)
        {
            int currentLeftIndex = left.GetFurthestRightIndex();
            int currentRightIndex = right.GetFurthestLeftIndex();
            bool doOnce = true;
            bool leftIndexChanged = false;
            bool rightIndexChanged = false;
            int bottomLeft = -1;
            int bottomRight = -1;
            List<PointF> rightPoints = right.GetAllPoints();
            List<PointF> leftPoints = left.GetAllPoints();
            while (doOnce || leftIndexChanged || rightIndexChanged)
            {
                if (doOnce || rightIndexChanged)
                {
                    bottomLeft = currentLeftIndex;
                    while (FindSlope(rightPoints[currentRightIndex], leftPoints[left.GetNextIndex(bottomLeft)]) >
                            FindSlope(rightPoints[currentRightIndex], leftPoints[bottomLeft]))
                    {
                        bottomLeft = left.GetNextIndex(bottomLeft);
                    }
                    if (bottomLeft == currentLeftIndex)
                    {
                        leftIndexChanged = false;
                        rightIndexChanged = false;
                    }
                    else
                    {
                        leftIndexChanged = true;
                        currentLeftIndex = bottomLeft;
                    }
                }
                if (doOnce || leftIndexChanged)
                {
                    bottomRight = currentRightIndex;
                    while (FindSlope(leftPoints[currentLeftIndex], rightPoints[right.GetPreviousIndex(bottomRight)]) <
                            FindSlope(leftPoints[currentLeftIndex], rightPoints[bottomRight]))
                    {
                        bottomRight = right.GetPreviousIndex(bottomRight);
                    }
                    if (bottomRight == currentRightIndex)
                    {
                        leftIndexChanged = false;
                        rightIndexChanged = false;
                    }
                    else
                    {
                        rightIndexChanged = true;
                        currentRightIndex = bottomRight;
                    }
                }
                doOnce = false;
            }
            return new KeyValuePair<int, int>(bottomLeft, bottomRight);
        }

        // Simple slope math, made it easier to see what I was doing when comparing tangent lines
        public Double FindSlope(PointF left, PointF right)
        {
            return -(right.Y - left.Y) / (right.X - left.X);
        }
    }

    //A container for hulls. I should probably put this in a separate file but I haven't figured that out just yet in c#
    class ConvexHull
    {
        List<PointF> allPoints;
        PointF farthestRight;

        //default constructor that should never be used
        private ConvexHull() { }

        //constructor I want to use
        public ConvexHull(List<PointF> points)
        {
            allPoints = points;
        }

        //Go to the next item  in the list. Accounts for the end of the list.
        public int GetNextIndex(int currentIndex)
        {
            if (currentIndex == allPoints.Count - 1)
            {
                return 0;
            }
            else
            {
                return currentIndex + 1;
            }
        }

        //Go to the previous item in the list. Accounts for the end of the list.
        public int GetPreviousIndex(int currentIndex)
        {
            if (currentIndex == 0)
            {
                return this.allPoints.Count - 1;
            }
            else
            {
                return currentIndex - 1;
            }
        }

        //Get all points from the list
        public List<PointF> GetAllPoints()
        {
            return this.allPoints;
        }

        public void SetFarthestRight(PointF point)
        {
            this.farthestRight = point;
        }

        //finds the largest x value in all the points from the list
        public int GetFurthestRightIndex()
        {
            int max = 0;
            for (int i = 0; i < allPoints.Count; i++)
            {
                if (allPoints[i].X > allPoints[max].X)
                {
                    max = i;
                }
            }
            return max;
        }

        //finds the smallest x value in all the points from the list
        public int GetFurthestLeftIndex()
        {
            int min = 0;
            for (int i = 0; i < allPoints.Count; i++)
            {
                if (allPoints[i].X < allPoints[min].X)
                {
                    min = i;
                }
            }
            return min;
        }
    }
}