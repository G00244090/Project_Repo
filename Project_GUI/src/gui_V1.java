import net.miginfocom.swing.MigLayout;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;


public class gui_V1  extends JPanel
{
    int press_Cnt = 0;
    String gift_code_output;
    String blank1 = "";
    String blank2 = "";
    String blank3 = "";
    String blank4 = "";
    String blank5 = "";
    String blank6 = "";
    String blank7 = "";
    String blank8 = "";
    String blank9 = "";
    String blank10 = "";
    String blank11 = "";
    String blank12 = "";
    String blank13 = "";
    String blank14 = "";

    public static void main(String[] args)
    {
        JFrame frame = new JFrame("Speed limit update v1.0");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.getContentPane().add(new gui_V1());
        frame.pack();
        frame.setVisible(true);
    }

    public gui_V1()
    {
        /*Matching panel creation*/
        setPreferredSize(new Dimension(800,600));
        setLayout(null);
        setBackground(Color.lightGray);

        //setBorder(BorderFactory.createTitledBorder("multiple_choice"));
        setLayout(new MigLayout("debug", "20[grow]10[grow]20 ","40[] []20[] [] [] [] [] []20[] [] []"));

        /* Label definitions*/
        JLabel Select_County = new JLabel("Select County");
        JLabel Select_type_road = new JLabel("Select Type Of Road");
        JLabel Roads_In_Area= new JLabel("List Of Roads Of Selected Type In Area");
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
        JTextArea County_TextArea = new JTextArea(12,40);
        JTextArea Road_Type_TextArea = new JTextArea(12,40);
        JTextArea Selected_Type_TextArea = new JTextArea(12,40);
        JTextArea Speed_TextArea = new JTextArea(10,40);
        JTextArea Information_TextArea = new JTextArea(10,40);

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
        County_Panel.setLayout(new MigLayout("debug", "[]"));
        JScrollPane Country_Scroll = new JScrollPane(County_Panel);
       // Country_Scroll.setPreferredSize(new Dimension(500,40));
        Country_Scroll.setVerticalScrollBarPolicy(Country_Scroll.VERTICAL_SCROLLBAR_ALWAYS);

        Road_Type_Panel.setLayout(new MigLayout("debug", "[]"));
        JScrollPane Road_Type_Scroll = new JScrollPane(Road_Type_Panel);
        // Road_Type_Scroll.setPreferredSize(new Dimension(500,40));
        Road_Type_Scroll.setVerticalScrollBarPolicy(Road_Type_Scroll.VERTICAL_SCROLLBAR_ALWAYS);

        Roads_In_Area_Panel.setLayout(new MigLayout("debug", "[]"));
        JScrollPane Roads_In_Area_scroll = new JScrollPane(Roads_In_Area_Panel);
        // Roads_In_Area_scroll.setPreferredSize(new Dimension(500,40));
        Roads_In_Area_scroll.setVerticalScrollBarPolicy(Roads_In_Area_scroll.VERTICAL_SCROLLBAR_ALWAYS);

        add(Select_County,"left,sg 3");
        add(Select_type_road,"left,sg 3,span");

        add(Country_Scroll,"left,pushx,growx,pushy,growy");
        add(Road_Type_Scroll,"left,pushx,growx,pushy,growy,span");

        County_Panel.add(County_TextArea, "left,pushx,growx,pushy,growy");
        Road_Type_Panel.add(Road_Type_TextArea,"left,pushx,growx,pushy,growy,span");
        Roads_In_Area_Panel.add(Selected_Type_TextArea,"left,pushx,growx,pushy,growy");

        add(Roads_In_Area,"left,sg 3");
        add(Select_Speed_Limit,"left sg 3,span");
        add(Roads_In_Area_scroll,"left,pushx,growx,pushy,growy");
        Speed_Limit_Panel.setLayout(new MigLayout("debug", "[][] ","[] [] [] [] [] [] []"));
        add(Speed_Limit_Panel,"pushx,pushy,growx,growy,span");

        Speed_Limit_Panel.add(Speed_125_kmh,"left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_100_kmh,"left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_80_kmh,"left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_60_kmh,"left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_50_kmh,"left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_30_kmh,"left,pushy,growy,wrap");
        Speed_Limit_Panel.add(Speed_Emergency_Stop,"left,pushy,growy,");
        Speed_Limit_Panel.add(Speed_Limit_Image_Panel,"left,span");
        add(Check_Current_Speed_Set,"left,span,wrap");
        add(Information,"left,span,wrap");
        add(Information_TextArea,"left,pushx,pushy,growx,growy,span");
        add(Update_Database_Button,"left");
        add(Broadcast_Speed_Change_Button,"left,span");

//        Road_Type_Panel.add(Add_Q_A,"left,sg 1,span");
//        Road_Type_Panel.add(Remove,"left,sg 1");
//        Roads_In_Area_Panel.add(Clear_Question_Text,"right,sg 1,wrap");
//        Roads_In_Area_Panel.add(Create_Gift_Code,"right,sg 1,span");
//        add(Gift_Output,"right,sg 2,split 2,span");
//
//        JScrollPane panelPane2 = new JScrollPane(Gift_Output_Text);
//        panelPane2.setPreferredSize(new Dimension(0,80));
//        panelPane2.setVerticalScrollBarPolicy(Country_Scroll.VERTICAL_SCROLLBAR_ALWAYS);
//        add(panelPane2, "pushx,growx,pushy,growy,wrap");
//        add(dummy2,"right,sg 2,split 2,span");
//        add(Clear_Gift_List,"right,wrap");
//



    }
}

