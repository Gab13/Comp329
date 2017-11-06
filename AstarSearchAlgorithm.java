package lejos.robotics.pathfinding;

import java.util.*;
import lejos.robotics.navigation.Waypoint;

/**
 * This is an implementation of the A* search algorithm. Typically this object would be instantiated and then used
 * in a NodePathFinder constructor, along with a set of connected nodes.
 * @see lejos.robotics.pathfinding.NodePathFinder
 * */

public class AstarSearchAlgorithm implements SearchAlgorithm{
	
	int main_loop = 0; 
	int neighbor_loop = 0;
	
	public Path findPath(Node start, Node goal) {
		
		// The set of nodes already evaluated. Empty at start.		
		ArrayList <Node> closedset = new ArrayList<Node>(); 
		
		// The set of tentative nodes to be evaluated. 
		ArrayList <Node> openset = new ArrayList<Node>(); 
		
		// openset contains startNode at start.
		openset.add(start); 
		
		// Distance from start along optimal path. Zero by definition since at start. g(start)
		start.setG_Score(0); 
		
		//start.setH_Score((float)Point2D.distance(start.x, start.y, goal.x, goal.y)); // ORIGINAL h(start)
		start.setH_Score(start.calculateH(goal)); 
		
		
		while (!openset.isEmpty()) {
			 // get the node in openset having the lowest f_score[] value
			Node x = getSafePoint(openset);

			main_loop++;
			
			if(x == goal) {
				Path final_path = new Path();
				
				reconstructPath(goal, start, final_path);
				
				return final_path;
			}
			// remove x from openset
			openset.remove(x); 
			
			// add x to closedset
			closedset.add(x); 

			Collection <Node> yColl = x.getNeighbors();
			Iterator <Node> yIter = yColl.iterator();

			while(yIter.hasNext()) { 
				// for each y in neighbor_nodes(x)
				
				neighbor_loop++; 
				Node y = yIter.next();
				
				// if y in closedset already, go to next one
				if(closedset.contains(y)) continue; 

				
				float tentative_g_score = x.getG_Score() + x.calculateG(y);
				boolean tentative_is_better = false;

				if (!openset.contains(y)) { 
					// if y not in openset
					
					// add y to openset
					openset.add(y); 					
					tentative_is_better = true;
					
				} else if(tentative_g_score < y.getG_Score()) { 
					// if tentative_g_score < g_score[y]
					
					tentative_is_better = true;
				} else
					tentative_is_better = false;

				if (tentative_is_better) {
					y.setPredecessor(x); 
				}

				y.setG_Score(tentative_g_score);
				y.setH_Score(y.calculateH(goal)); 
				
			} // while yIter.hasNext()
		} // while main loop
		
		return null; // returns null if fails to find a  continuous path.
	}

	/**
	 * Finds the node within a set of neighbors with the least cost (potentially shortest distance to goal). 
	 * @return The node with the least cost.
	 */
	private static Node getSafePoint(Collection <Node> nodeSet) {
		
		Iterator <Node> nodeIterator = nodeSet.iterator();
		Node best = nodeIterator.next();
		
		while(nodeIterator.hasNext()) {
			Node cur = nodeIterator.next();
			
			if(cur.getF_Score() < best.getF_Score())
				best = cur;
		}
		return best;
	}
	
	/**
	 * Given the current node and the start node, this method retraces the completed path. It relies
	 * on Node.getPredecessor() to backtrack using recursion. 
	 * 
	 * @param current_node
	 * @param start
	 * @param path The path output by this algorithm.
	 */
	private static final void reconstructPath(Node current_node, Node start, Collection <Waypoint> path){
		if(current_node != start)
			reconstructPath(current_node.getPredecessor(), start, path);
		path.add(new Waypoint(current_node.x, current_node.y));
		return;
	}
	

}