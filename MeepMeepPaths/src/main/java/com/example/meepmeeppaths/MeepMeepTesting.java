package com.example.meepmeeppaths;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class MeepMeepTesting {

	public static void main( String[] args ) {
		MeepMeep meepMeep = new MeepMeep( 600 );

		RoadRunnerBotEntity myBot = new DefaultBotBuilder( meepMeep )
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints( 30, 30, Math.toRadians( 60 ), Math.toRadians( 60 ), 8.6 )
				.followTrajectorySequence( drive -> {
							try {
								return ((MeepMeepPath) Class.forName( args[0] ).newInstance( )).getTrajectorySequence( drive );
							} catch( InstantiationException | IllegalAccessException | ClassNotFoundException e ) {
								e.printStackTrace( );
							}
							return new TrajectorySequence( new ArrayList<>( ) );
						}
				);

		meepMeep.setBackground( MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL )
				.setDarkMode( true )
				.setBackgroundAlpha( 0.95f )
				.addEntity( myBot )
				.start( );
	}
}