package org.firstinspires.ftc.teamcode.utils.audio;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class SoundLibrary {

	private static HardwareMap hardwareMap;

	private static boolean finishedInit = false;

	public /*private*/ static final ArrayList<Audio> audioList = new ArrayList<>( );

	public SoundLibrary( HardwareMap hardwareMap ) {

		SoundLibrary.hardwareMap = hardwareMap;

		initSounds( );
	}

	private void initSounds( ) {

		String[] soundNames = new String[]{
				"falcon_punch_smash",
				"fine_addition",
				"gamecube_startup",
//				"gold",
				"hallelujah_chorus",
				"have_hulk",
				"hello_there_startup",
				"my_precious",
				"nooo",
				"ps_startup",
				"punch",
//				"seismic_charge_smash",
//				"silver",
				"slurp_yummy",
				"smash",
				"windows_startup" };

		float[] volumes = new float[]{ /*0.5f*/1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f };

		// other audios
		String names = toString( );
		for( int i = 0; i < soundNames.length; i++ )
			if( !names.contains( soundNames[i] ) )
				audioList.add( new Audio( hardwareMap, soundNames[i], volumes[i] ) );

		//audioList.add( new Audio( hardwareMap, "" ) );

		// checks all of the sounds and removes the ones that aren't found
		for( int i = 0; i < audioList.size( ); i++ )
			if( !audioList.get( i ).found( ) )
				audioList.remove( i-- );

		finishedInit = true;
	}

	public static boolean finishedInit( ) {
		return finishedInit;
	}

	/**
	 * @param audioName the name of the audio to play
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playAudio( String audioName ) {
		for( int i = 0; i < audioList.size( ); i++ ) {
			if( audioList.get( i ).getName( ).equals( audioName ) ) {
				audioList.get( i ).play( );
				return "Audio \"" + audioName + "\" :: playing";
			}
		}
		return "Audio \"" + audioName + "\" :: not found";
	}

//	public static AudioPlayer getAudioPlayer( String audioName ) {
//		for( int i = 0; i < audioList.size( ); i++ )
//			if( audioList.get( i ).getName( ).equals( audioName ) )
//				return audioList.get( i );
//		return null;
//	}

	/**
	 * plays the playstation startup (the audio file with the name "ps_startup")
	 *
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playStartup( ) {
		return playAudio( "ps_startup" );
	}

	/**
	 * plays a random startup (any audio file containing the name "startup")
	 *
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playRandomStartup( ) {
		return playRandomAudioOfType( "startup" );
	}

	/**
	 * plays a random smash (any audio file containing the name "smash")
	 *
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playRandomSmash( ) {
		return playRandomAudioOfType( "smash" );
	}

	/**
	 * plays a random sound loaded
	 *
	 * @return a string saying whether the audio is playing or was not found
	 */
	public static String playRandomSound( ) {
		int randomPos = (int) (Math.random( ) * audioList.size( ));
		return playAudio( audioList.get( randomPos ).getName( ) );
	}

	/**
	 * @return a list with all the loaded audio names
	 */
	public static String[] getAudioNames( ) {
		String[] audios = new String[audioList.size( )];

		for( int i = 0; i < audioList.size( ); i++ )
			audios[i] = audioList.get( i ).getName( );

		return audios;
	}

	/**
	 * @return a string with a list of all the names separated by line separators & dashes
	 */
	public String toString( ) {
		String audios = "", listSeparator = "\n";

		for( int i = 0; i < audioList.size( ); i++ )
			audios += "- " + audioList.get( i ).getName( ) + (i != audioList.size( ) - 1 ? listSeparator : "");

		return audios;
	}

	public static void stopAllAudios( ) {
		Audio.stopAllAudios( );
	}

	/**
	 * @param audioType the string to look for audios containing it
	 * @return a string saying whether the audio is playing or was not found
	 */
	private static String playRandomAudioOfType( String audioType ) {
		ArrayList<Audio> list = new ArrayList<>( );
		for( int i = 0; i < audioList.size( ); i++ )
			if( audioList.get( i ).getName( ).contains( audioType ) )
				list.add( audioList.get( i ) );
		// changed this
		if( list.size( ) > 0 )
			return playAudio( list.get( (int) (Math.random( ) * list.size( )) ).getName( ) );
		return "No " + audioType + " audio found";
	}
}
