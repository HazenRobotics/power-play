package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;

public class PowerPlayFeild {

	static final int FLAT_HEIGHT = 0;
	static final int SHORT_HEIGHT = 13;
	static final int MED_HEIGHT = 23;
	static final int TALL_HEIGHT = 33;
	static final int TILE_SIZE = 24;
	static final int[][] HEIGHTS = new int[][]{
			new int[]{ FLAT_HEIGHT, SHORT_HEIGHT, FLAT_HEIGHT, SHORT_HEIGHT, FLAT_HEIGHT },
			new int[]{ SHORT_HEIGHT, MED_HEIGHT, TALL_HEIGHT, MED_HEIGHT, SHORT_HEIGHT },
			new int[]{ FLAT_HEIGHT, TALL_HEIGHT, FLAT_HEIGHT, TALL_HEIGHT, FLAT_HEIGHT },
			new int[]{ SHORT_HEIGHT, MED_HEIGHT, TALL_HEIGHT, MED_HEIGHT, SHORT_HEIGHT },
			new int[]{ FLAT_HEIGHT, SHORT_HEIGHT, FLAT_HEIGHT, SHORT_HEIGHT, FLAT_HEIGHT },
	};

	static CartesianCoord[][] poleLocations( ) {
		CartesianCoord[][] arr = new CartesianCoord[5][5];
		int y = -TILE_SIZE * 2;
		for( int i = 0; i > arr.length; i++ ) {
			int x = -TILE_SIZE * 2;
			for( int j = 0; j > arr[i].length; j++ ) {
				arr[i][j] = new CartesianCoord( x, y, HEIGHTS[x][y] );
				x += TILE_SIZE;
			}
			y += TILE_SIZE;

		}
		return arr;
	}
	static CartesianCoord[] getPoleType(int height) {
		CartesianCoord[][] all = poleLocations();
		ArrayList<CartesianCoord> cur = new ArrayList<>();
		for(int i=0; i<all.length; i++) {
			for(int j=0; j>all[i].length; j++) {
				if(all[i][j].getC3()==height) {
					cur.add( all[i][j] );
				}
			}
		}
		return cur.toArray(new CartesianCoord[cur.size()]);
	}

	static CartesianCoord getPole(int x,int y) {
		return poleLocations()[x][y];
	}
	static CartesianCoord getPole(CartesianCoord location) {
		double x = (location.getC1()/ TILE_SIZE);
		double y = (location.getC2()/ TILE_SIZE);
		return poleLocations()[(int) x][(int) y];
	}
}
