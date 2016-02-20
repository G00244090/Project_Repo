import net.miginfocom.swing.MigLayout;
import javax.swing.*;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.SQLException;


public class gui_V1  extends JPanel {
    String image = "30 Kmh small";
    String limit = "30 Km/h";
    JLabel picLabel = new JLabel(new ImageIcon());
    private PrintWriter output; // output stream to client
    private BufferedReader input; // input stream from client
    private ServerSocket server; // server socket
    private Socket connection; // connection to client
    private int counter = 1; // counter of number of connections
    JTextArea Information_TextArea = new JTextArea(10, 40);


    public static void main(String[] args) {
        JFrame frame = new JFrame("Speed limit update v1.0");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        gui_V1 gi = new gui_V1();
        frame.getContentPane().add(gi);
        frame.pack();
        frame.setVisible(true);
        gi.runServer();
    }

    public gui_V1() {
        /*Matching panel creation*/
        setPreferredSize(new Dimension(900, 800));
        setLayout(null);
        setBackground(Color.lightGray);

        //setBorder(BorderFactory.createTitledBorder("multiple_choice"));
        setLayout(new MigLayout("", "20[]10[]20 ", "40[] []20[] [] [] [] [] []20[] [] []"));

        /* Label definitions*/
        JLabel Select_County = new JLabel("Select County");
        JLabel Select_type_road = new JLabel("Select Type Of Road");
        JLabel Roads_In_Area = new JLabel("List Of Roads Of Selected Type In Area");
        JLabel Select_Speed_Limit = new JLabel("Select Speed Limit To Set");
        JLabel Information = new JLabel("Information");
         /* Panel definitions*/
        JPanel County_Panel = new JPanel();
        JPanel Road_Type_Panel = new JPanel();
        JPanel Roads_In_Area_Panel = new JPanel();
        JPanel Speed_Limit_Panel = new JPanel();
        JPanel Speed_Limit_Image_Panel = new JPanel();
        JLabel Gift_Output = new JLabel("Gift Output:");
         /* TextArea definitions*/
        String[] empty = {"Empty"};
        String[] county = {"Mayo", "Galway", "Dublin"};
        String[] Road_Type = {"National Route", "Motorway", "Regional"};

        String[] Mayo_National = {"Mo", "N58", "N59", "N60", "N83", "N84"};
        String[] Mayo_Motorway = {"Mo", "M58", "M59", "M60", "M83", "M84"};
        String[] Mayo_Regional = {"Mo", "R293", "R294", "R297", "R300", "R310"};

        String[] Galway_National = {"G", "N59", "N60", "N83", "N84"};
        String[] Galway_Motorway = {"G", "M58", "M59", "M60", "M83", "M84"};
        String[] Galway_Regional = {"G", "R293", "R294", "R297", "R300", "R310"};

        String[] Dublin_National = {"D", "N59", "N60", "N83", "N84"};
        String[] Dublin_Motorway = {"D", "M58", "M59", "M60", "M83", "M84"};
        String[] Dublin_Regional = {"D", "R293", "R294", "R297", "R300", "R310"};

        JList Selected_Type_TextArea;
        JList County_TextArea = new JList(county);
        JList Road_Type_TextArea = new JList(Road_Type);
        Selected_Type_TextArea = new JList(empty);


        JTextArea Speed_TextArea = new JTextArea(10, 40);


        /*Speed limit radio buttons*/
        JRadioButton Speed_125_kmh = new JRadioButton("120 km/h");
        JRadioButton Speed_100_kmh = new JRadioButton("100 km/h");
        JRadioButton Speed_80_kmh = new JRadioButton("80 km/h");
        JRadioButton Speed_60_kmh = new JRadioButton("60 km/h");
        JRadioButton Speed_50_kmh = new JRadioButton("50 km/h");
        JRadioButton Speed_30_kmh = new JRadioButton("30 km/h");
        JRadioButton Speed_Emergency_Stop = new JRadioButton("Emergency Stop");
        /*Buttons */
        JButton Check_Current_Speed_Set = new JButton("Check current speed set");
        JButton Update_Database_Button = new JButton("Update Database");
        JButton Broadcast_Speed_Change_Button = new JButton("Broadcast Speed Change");


        /*panel additions*/
        County_Panel.setLayout(new MigLayout("", "[]"));
        JScrollPane Country_Scroll = new JScrollPane(County_Panel);
        Country_Scroll.setVerticalScrollBarPolicy(Country_Scroll.VERTICAL_SCROLLBAR_ALWAYS);

        Road_Type_Panel.setLayout(new MigLayout("", "[]"));
        JScrollPane Road_Type_Scroll = new JScrollPane(Road_Type_Panel);
        Road_Type_Scroll.setVerticalScrollBarPolicy(Road_Type_Scroll.VERTICAL_SCROLLBAR_ALWAYS);

        Roads_In_Area_Panel.setLayout(new MigLayout("", "[]"));
        JScrollPane Roads_In_Area_scroll = new JScrollPane(Roads_In_Area_Panel);
        Roads_In_Area_scroll.setVerticalScrollBarPolicy(Roads_In_Area_scroll.VERTICAL_SCROLLBAR_ALWAYS);

        add(Select_County, "left,sg 3");
        add(Select_type_road, "left,sg 3,span");
        add(Country_Scroll, "left,pushx,growx,pushy,growy");
        add(Road_Type_Scroll, "left,pushx,growx,pushy,growy,span");
        County_Panel.add(County_TextArea, "left,pushx,growx,pushy,growy");
        County_TextArea.setSelectionMode(ListSelectionModel.SINGLE_INTERVAL_SELECTION);
        County_TextArea.setLayoutOrientation(JList.HORIZONTAL_WRAP);

        Road_Type_Panel.add(Road_Type_TextArea, "left,pushx,growx,pushy,growy,span");
        Roads_In_Area_Panel.add(Selected_Type_TextArea, "left,pushx,growx,pushy,growy");

        add(Roads_In_Area, "left,sg 3");
        add(Select_Speed_Limit, "left sg 3,span");
        add(Roads_In_Area_scroll, "left,pushx,growx,pushy,growy");
        Speed_Limit_Panel.setLayout(new MigLayout("", "[grow][grow] ", "[] [] [] [] [] [] []"));
        add(Speed_Limit_Panel, "pushx,pushy,growx,growy,span");

        Speed_Limit_Panel.add(Speed_125_kmh, "left,wrap");
        Speed_Limit_Panel.add(Speed_Limit_Image_Panel, "Cell 1 0,spany,spany,wrap");
        Speed_Limit_Image_Panel.setMaximumSize(new Dimension(200, 200));
        Speed_Limit_Panel.add(Speed_100_kmh, "left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_80_kmh, "left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_60_kmh, "left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_50_kmh, "left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_30_kmh, "left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_Emergency_Stop, "left,wrap");
        //Adds the image to the image panel
        image = "30 Kmh small";
        picLabel = new JLabel(new ImageIcon("images/speed_limits/" + image + ".png"));
        Speed_Limit_Image_Panel.add(picLabel, "push,grow");
        Speed_Limit_Image_Panel.revalidate();

        add(Check_Current_Speed_Set, "left,span,wrap");
        add(Information, "left,span,wrap");
        add(Information_TextArea, "left,pushx,pushy,growx,growy,span");
        add(Update_Database_Button, "left,split2");
        add(Broadcast_Speed_Change_Button, "left");
        /*Below are the action listeners */

        Road_Type_TextArea.addListSelectionListener(new ListSelectionListener() {
            @Override
            public void valueChanged(ListSelectionEvent e) {
                if (County_TextArea.getSelectedValue().toString().equalsIgnoreCase("Mayo")) {
                    if (Road_Type_TextArea.getSelectedValue().toString().equalsIgnoreCase("National Route"))
                        Selected_Type_TextArea.setListData(Mayo_National);
                    else if (Road_Type_TextArea.getSelectedValue().toString().equalsIgnoreCase("Regional"))
                        Selected_Type_TextArea.setListData(Mayo_Regional);
                    else if (Road_Type_TextArea.getSelectedValue().toString().equalsIgnoreCase("Motorway"))
                        Selected_Type_TextArea.setListData(Mayo_Motorway);
                }
                if (County_TextArea.getSelectedValue().toString().equalsIgnoreCase("Galway")) {
                    if (Road_Type_TextArea.getSelectedValue().toString().equalsIgnoreCase("National Route"))
                        Selected_Type_TextArea.setListData(Galway_National);
                    else if (Road_Type_TextArea.getSelectedValue().toString().equalsIgnoreCase("Regional"))
                        Selected_Type_TextArea.setListData(Galway_Regional);
                    else if (Road_Type_TextArea.getSelectedValue().toString().equalsIgnoreCase("Motorway"))
                        Selected_Type_TextArea.setListData(Galway_Motorway);
                }
                if (County_TextArea.getSelectedValue().toString().equalsIgnoreCase("Dublin")) {
                    if (Road_Type_TextArea.getSelectedValue().toString().equalsIgnoreCase("National Route"))
                        Selected_Type_TextArea.setListData(Dublin_National);
                    else if (Road_Type_TextArea.getSelectedValue().toString().equalsIgnoreCase("Regional"))
                        Selected_Type_TextArea.setListData(Dublin_Regional);
                    else if (Road_Type_TextArea.getSelectedValue().toString().equalsIgnoreCase("Motorway"))
                        Selected_Type_TextArea.setListData(Dublin_Motorway);
                }
            }
        });


        Speed_30_kmh.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                Speed_50_kmh.setSelected(false);
                Speed_60_kmh.setSelected(false);
                Speed_80_kmh.setSelected(false);
                Speed_100_kmh.setSelected(false);
                Speed_125_kmh.setSelected(false);
                Speed_Emergency_Stop.setSelected(false);
                Information_TextArea.setText("");
                image = "30 Kmh small";
                limit = "30 Km/h";
                Information_TextArea.append(County_TextArea.getSelectedValue().toString() + " "
                        + Road_Type_TextArea.getSelectedValue().toString() + " " +
                        Selected_Type_TextArea.getSelectedValue().toString() + " " + "30 Km/h Update");//Selected_Type_TextArea.getSelectedValue().toString()+
                picLabel.setIcon(new ImageIcon("images/speed_limits/" + image + ".png"));
            }
        });
        Speed_50_kmh.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                Speed_30_kmh.setSelected(false);
                Speed_60_kmh.setSelected(false);
                Speed_80_kmh.setSelected(false);
                Speed_100_kmh.setSelected(false);
                Speed_125_kmh.setSelected(false);
                Speed_Emergency_Stop.setSelected(false);
                Information_TextArea.setText("");
                image = "50 Kmh small";
                limit = "50 Km/h";
                Information_TextArea.append(County_TextArea.getSelectedValue().toString() + " "
                        + Road_Type_TextArea.getSelectedValue().toString() + " " +
                        Selected_Type_TextArea.getSelectedValue().toString() + " " + "50 Km/h Update");
                picLabel.setIcon(new ImageIcon("images/speed_limits/" + image + ".png"));
                Speed_Limit_Image_Panel.add(picLabel, "push,grow");
//                Speed_Limit_Image_Panel.repaint();
            }
        });
        Speed_60_kmh.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                Speed_30_kmh.setSelected(false);
                Speed_50_kmh.setSelected(false);
                Speed_80_kmh.setSelected(false);
                Speed_100_kmh.setSelected(false);
                Speed_125_kmh.setSelected(false);
                Speed_Emergency_Stop.setSelected(false);
                Information_TextArea.setText("");
                image = "60 Kmh small";
                Information_TextArea.append(County_TextArea.getSelectedValue().toString() + " "
                        + Road_Type_TextArea.getSelectedValue().toString() + " " +
                        Selected_Type_TextArea.getSelectedValue().toString() + "" + "60 Km/h Update");
                picLabel = new JLabel(new ImageIcon("images/speed_limits/" + image + ".png"));
                // Speed_Limit_Image_Panel.add(picLabel, "push,grow");
//                Speed_Limit_Image_Panel.revalidate();
            }
        });
        Speed_80_kmh.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                Speed_30_kmh.setSelected(false);
                Speed_60_kmh.setSelected(false);
                Speed_50_kmh.setSelected(false);
                Speed_100_kmh.setSelected(false);
                Speed_125_kmh.setSelected(false);
                Speed_Emergency_Stop.setSelected(false);
                Information_TextArea.setText("");
                image = "80 Kmh small";
                limit = "80 Km/h";
                Information_TextArea.append(County_TextArea.getSelectedValue().toString() + " "
                        + Road_Type_TextArea.getSelectedValue().toString() + " " +
                        Selected_Type_TextArea.getSelectedValue().toString() + " " + "80 Km/h Update");
                picLabel = new JLabel(new ImageIcon("images/speed_limits/" + image + ".png"));
                //Speed_Limit_Image_Panel.add(picLabel, "push,grow");
//                Speed_Limit_Image_Panel.revalidate();
            }
        });
        Speed_100_kmh.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                Speed_30_kmh.setSelected(false);
                Speed_60_kmh.setSelected(false);
                Speed_50_kmh.setSelected(false);
                Speed_80_kmh.setSelected(false);
                Speed_125_kmh.setSelected(false);
                Speed_Emergency_Stop.setSelected(false);
                Information_TextArea.setText("");
                image = "100 Kmh small";
                limit = "100 Km/h";
                Information_TextArea.append(County_TextArea.getSelectedValue().toString() + " "
                        + Road_Type_TextArea.getSelectedValue().toString() + " " +
                        Selected_Type_TextArea.getSelectedValue().toString() + "" + "100 Km/h Update");

                picLabel.setIcon(new ImageIcon("images/speed_limits/" + image + ".png"));
                // Speed_Limit_Image_Panel.add(picLabel, "push,grow");
//                Speed_Limit_Image_Panel.revalidate();
            }
        });
        Speed_125_kmh.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                Speed_30_kmh.setSelected(false);
                Speed_60_kmh.setSelected(false);
                Speed_50_kmh.setSelected(false);
                Speed_80_kmh.setSelected(false);
                Speed_100_kmh.setSelected(false);
                Speed_Emergency_Stop.setSelected(false);
                Information_TextArea.setText("");
                image = "120 Kmh small";
                limit = "120 Km/h";
                Information_TextArea.append(County_TextArea.getSelectedValue().toString() + " "
                        + Road_Type_TextArea.getSelectedValue().toString() + " " +
                        Selected_Type_TextArea.getSelectedValue().toString() + "" + "120 Km/h Update");
                picLabel.setIcon(new ImageIcon("images/speed_limits/" + image + ".png"));
                //Speed_Limit_Image_Panel.add(picLabel, "push,grow");
//                Speed_Limit_Image_Panel.revalidate();
            }
        });
        Speed_Emergency_Stop.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                Speed_30_kmh.setSelected(false);
                Speed_60_kmh.setSelected(false);
                Speed_50_kmh.setSelected(false);
                Speed_80_kmh.setSelected(false);
                Speed_100_kmh.setSelected(false);
                Speed_125_kmh.setSelected(false);
                Information_TextArea.setText("");
                Information_TextArea.append(County_TextArea.getSelectedValue().toString() + " "
                        + Road_Type_TextArea.getSelectedValue().toString() + " " + "Emergency:Proceed with caution !!!");
                picLabel.setIcon(new ImageIcon("images/speed_limits/" + image + ".png"));
                Speed_Limit_Image_Panel.add(picLabel, "push,grow");
                Speed_Limit_Image_Panel.revalidate();
            }
        });
        Update_Database_Button.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                // Connection mon_connect;
                String driver = "com.mysql.jdbc.Driver";
                try {
                    Class.forName(driver);
                    String url = "jdbc:mysql://localhost:3306/roadnetwork"; //Database name here
                    Connection mon_connect = DriverManager.getConnection(url, "root", "root");  //URL, user and password
                    System.out.println("" + County_TextArea.getSelectedValue().toString());
                    System.out.println("" + limit);
                    System.out.println("" + Selected_Type_TextArea.getSelectedValue().toString());
                    PreparedStatement send_data = mon_connect.prepareStatement("UPDATE " + County_TextArea.getSelectedValue().toString() + " Set Speedlimit=? WHERE RoadName=?;");
                    send_data.setString(1, limit);
                    send_data.setString(2, Selected_Type_TextArea.getSelectedValue().toString());
                    int catch_return = send_data.executeUpdate();

                } catch (ClassNotFoundException de) {
                    de.printStackTrace();
                } catch (SQLException se) {
                    se.printStackTrace();
                }
            }
        });
        Broadcast_Speed_Change_Button.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                sendData("testing 1 2 3 ");
                // runServer();
            }
        });

    }
    // set up and run server
    public void runServer()
    {
        try // set up server that recives a connaction on port 27 
        {
            server = new ServerSocket( 27, 100 ); // Creating the socket connection, listening to port 27

            while (true )
            {
                try
                {
                    ClientConnection(); // wait until a client has connection
                    StreamsSetup(); // Set up Server data input & output streams
                    processConnection(); // process connection
                } // end try
                catch ( EOFException eofException )
                {
                    Information_TextArea.append( "\nServer terminated connection" );
                } // end catch
                finally
                {
                    //closeConnection(); //  close connection
                    counter++;
                } // end finally
            } // end while
        } // end try
        catch ( IOException ioException )
        {
            ioException.printStackTrace();
        } // end catch
    } // end method runServer

    // wait for connection, recieve client information
    private void ClientConnection() throws IOException
    {
        Information_TextArea.append( "Waiting for the Client to Connect\n" );
        connection = server.accept(); // allow server to accept connection
        Information_TextArea.append( "Client Connection " + counter + " received from: " +
                connection.getInetAddress().getHostName() );
    } // end method ClientConnection

    // Sets up the data streams for the clients
    private void StreamsSetup() throws IOException
    {
        // set up output stream for objects
        output = new PrintWriter( connection.getOutputStream() );
        output.flush(); // flush output buffer to send header information

        // set up input stream for objects
        input = new BufferedReader( new InputStreamReader(connection.getInputStream()) );

        Information_TextArea.append( "\nStreams have been set up\n" );
    } // end method StreamsSetup

    // process connection with client
    private void processConnection() throws IOException
    {
        Information_TextArea.append( "Setup Complete\n" );
        String message = "Successfull connection to server";
        sendData( message ); // send connection message
        Information_TextArea.append( "Connection Message Sent\n" );
        do // process messages sent from client
        {
            try // read message and display it
            {
                String line = input.readLine();
                Information_TextArea.append("Client Response:\n" +  input.readLine());
                message = (line.toString()); // read new message
                Information_TextArea.append( "\n" + message ); // display message
            } // end try
            catch ( IOException ioException )
            {
                Information_TextArea.append( "\nUnknown object type received" );
            } // end catch

        } while ( !message.equals( "TERMINATE CONNECTION 4321" ) );
    } // end method processConnection

    // close streams and socket
    private void closeConnection()
    {
        try
        {
            output.close(); // close output stream
            input.close(); // close input stream
            connection.close(); // close socket
        } // end try
        catch ( IOException ioException )
        {
            ioException.printStackTrace();
        } // end catch
    } // end method closeConnection

    // send message to client
    private void sendData( String message )
    {
        try // send object to client
        {
            output.print( "<UPDATE>" + message );
            output.flush(); // flush output to client
        } // end try
        catch (Exception Io )
        {
            Information_TextArea.append("\nException Thrown");
        } // end catch
    } // end method sendData
}

