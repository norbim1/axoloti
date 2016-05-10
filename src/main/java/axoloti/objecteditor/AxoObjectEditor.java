/**
 * Copyright (C) 2013 - 2016 Johannes Taelman
 *
 * This file is part of Axoloti.
 *
 * Axoloti is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Axoloti is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Axoloti. If not, see <http://www.gnu.org/licenses/>.
 */
package axoloti.objecteditor;

import axoloti.DocumentWindow;
import axoloti.DocumentWindowList;
import axoloti.MainFrame;
import axoloti.object.AxoObject;
import axoloti.object.AxoObjectInstance;
import axoloti.object.ObjectModifiedListener;
import axoloti.utils.AxolotiLibrary;
import java.awt.BorderLayout;
import java.awt.Point;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.DefaultListModel;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import org.fife.ui.rsyntaxtextarea.RSyntaxTextArea;
import org.fife.ui.rsyntaxtextarea.SyntaxConstants;
import org.fife.ui.rtextarea.RTextScrollPane;
import org.simpleframework.xml.Serializer;
import org.simpleframework.xml.core.Persister;

/**
 *
 * @author Johannes Taelman
 */
public final class AxoObjectEditor extends JFrame implements DocumentWindow, ObjectModifiedListener {

    AxoObject editObj;
    final AxoObject origObj;
    private final RSyntaxTextArea jTextAreaLocalData;
    private final RSyntaxTextArea jTextAreaInitCode;
    private final RSyntaxTextArea jTextAreaKRateCode;
    private final RSyntaxTextArea jTextAreaSRateCode;
    private final RSyntaxTextArea jTextAreaDisposeCode;
    private final RSyntaxTextArea jTextAreaMidiCode;
    boolean modified;

    static RSyntaxTextArea initCodeEditor(JPanel p) {
        RSyntaxTextArea rsta = new RSyntaxTextArea(20, 60);
        rsta.setSyntaxEditingStyle(SyntaxConstants.SYNTAX_STYLE_CPLUSPLUS);
        rsta.setCodeFoldingEnabled(true);
        RTextScrollPane sp = new RTextScrollPane(rsta);
        p.setLayout(new BorderLayout());
        p.add(sp);
        rsta.setVisible(true);
        return rsta;
    }

    public AxoObjectEditor(final AxoObject origObj, boolean editOriginal) {
        initComponents();
        fileMenu1.initComponents();
        DocumentWindowList.RegisterWindow(this);
        jTextAreaLocalData = initCodeEditor(jPanelLocalData);
        jTextAreaInitCode = initCodeEditor(jPanelInitCode);
        jTextAreaKRateCode = initCodeEditor(jPanelKRateCode2);
        jTextAreaSRateCode = initCodeEditor(jPanelSRateCode);
        jTextAreaDisposeCode = initCodeEditor(jPanelDisposeCode);
        jTextAreaMidiCode = initCodeEditor(jPanelMidiCode2);
        setIconImage(new ImageIcon(getClass().getResource("/resources/axoloti_icon.png")).getImage());
        setTitle(origObj.id);
        this.origObj = origObj;
        if (editOriginal) {
            this.editObj = origObj;
        }
        else {
            try {
                this.editObj = origObj.clone();
            } catch (CloneNotSupportedException ex) {
                Logger.getLogger(AxoObjectEditor.class.getName()).log(Level.SEVERE, null, ex);
                this.editObj = new AxoObject();
            }
        }
        editObj.addObjectModifiedListener(this);

        jLabelName.setText(editObj.getCName());
        jTextFieldAuthor.setText(editObj.sAuthor);
        jTextFieldAuthor.addFocusListener(new FocusListener() {
            @Override
            public void focusLost(FocusEvent e) {
                if (editObj.sAuthor == null || !editObj.sAuthor.equals(jTextFieldAuthor.getText())) {
                    editObj.sAuthor = jTextFieldAuthor.getText().trim();
                    FireObjectModified();
                }
            }

            @Override
            public void focusGained(FocusEvent e) {
            }
        });

        jTextFieldLicense.setText(editObj.sLicense);
        jTextFieldLicense.addFocusListener(new FocusListener() {
            @Override
            public void focusLost(FocusEvent e) {
                if (editObj.sLicense == null || !editObj.sLicense.equals(jTextFieldLicense.getText())) {
                    editObj.sLicense = jTextFieldLicense.getText().trim();
                    FireObjectModified();
                }
            }

            @Override
            public void focusGained(FocusEvent e) {
            }
        });
        jTextDesc.setText(editObj.sDescription);
        jTextDesc.addFocusListener(new FocusListener() {
            @Override
            public void focusLost(FocusEvent e) {
                if (editObj.sLicense == null || !editObj.sLicense.equals(jTextDesc.getText())) {
                    editObj.sDescription = jTextDesc.getText().trim();
                    FireObjectModified();
                }
            }

            @Override
            public void focusGained(FocusEvent e) {
            }
        });

        jLabelMidiPrototype.setText(AxoObjectInstance.MidiHandlerFunctionHeader);

        inletDefinitionsEditor1.initComponents(editObj);
        outletDefinitionsEditorPanel1.initComponents(editObj);
        paramDefinitionsEditorPanel1.initComponents(editObj);
        attributeDefinitionsEditorPanel1.initComponents(editObj);
        displayDefinitionsEditorPanel1.initComponents(editObj);

        if (editObj.includes != null) {
            for (String i : editObj.includes) {
                ((DefaultListModel) jListIncludes.getModel()).addElement(i);
            }
        }

        if (editObj.depends != null) {
            for (String i : editObj.depends) {
                ((DefaultListModel) jListDepends.getModel()).addElement(i);
            }
        }

        FocusListener fl = new FocusListener() {
            @Override
            public void focusGained(FocusEvent e) {
            }

            @Override
            public void focusLost(FocusEvent e) {
                applyChangesToEdit();
            }
        };

        jTextAreaLocalData.addFocusListener(fl);
        jTextAreaInitCode.addFocusListener(fl);
        jTextAreaKRateCode.addFocusListener(fl);
        jTextAreaSRateCode.addFocusListener(fl);
        jTextAreaDisposeCode.addFocusListener(fl);
        jTextAreaMidiCode.addFocusListener(fl);
        rSyntaxTextAreaXML.setEditable(false);
        FireObjectModified();
        modified = false;

        // is it from the factory?
        AxolotiLibrary sellib = null;
        for (AxolotiLibrary lib : MainFrame.prefs.getLibraries()) {
            if (editObj.sPath != null && editObj.sPath.startsWith(lib.getLocalLocation())) {

                if (sellib == null || sellib.getLocalLocation().length() < lib.getLocalLocation().length()) {
                    sellib = lib;
                }
            }
        }
        if (sellib != null) {
            jMenuItemSave.setEnabled(!sellib.isReadOnly());
            jMenuItemApply.setEnabled(!sellib.isReadOnly());
        }
        // embedded object
        if (editObj.sPath == null) {
            jMenuItemSave.setEnabled(false);
        }
    }

    void applyChangesToEdit() {
        editObj.sLocalData = jTextAreaLocalData.getText();
        editObj.sInitCode = jTextAreaInitCode.getText();
        editObj.sKRateCode = jTextAreaKRateCode.getText();
        editObj.sSRateCode = jTextAreaSRateCode.getText();
        editObj.sDisposeCode = jTextAreaDisposeCode.getText();
        editObj.sMidiCode = jTextAreaMidiCode.getText();
    }

    void applyChangesToOriginal() {
        applyChangesToEdit();
        origObj.sAuthor = editObj.sAuthor;
        origObj.sDescription = editObj.sDescription;
        origObj.sLicense = editObj.sLicense;
        origObj.helpPatch = editObj.helpPatch;
        origObj.sLocalData = editObj.sLocalData;
        origObj.sInitCode = editObj.sInitCode;
        origObj.sKRateCode = editObj.sKRateCode;
        origObj.sSRateCode = editObj.sSRateCode;
        origObj.sDisposeCode = editObj.sDisposeCode;
        origObj.sMidiCode = editObj.sMidiCode;
        origObj.inlets = editObj.inlets;
        origObj.outlets = editObj.outlets;
        origObj.includes = editObj.includes;
        origObj.depends = editObj.depends;
        origObj.displays = editObj.displays;
        origObj.attributes = editObj.attributes;
        origObj.params = editObj.params;
    }

    void FireObjectModified() {
        modified = true;
        jTextAreaLocalData.setText(editObj.sLocalData);
        jTextAreaInitCode.setText(editObj.sInitCode);
        jTextAreaKRateCode.setText(editObj.sKRateCode);
        jTextAreaSRateCode.setText(editObj.sSRateCode);
        jTextAreaDisposeCode.setText(editObj.sDisposeCode);
        jTextAreaMidiCode.setText(editObj.sMidiCode);
        Serializer serializer = new Persister();
        ByteArrayOutputStream os = new ByteArrayOutputStream(2048);
        try {
            serializer.write(editObj, os);
        } catch (Exception ex) {
            Logger.getLogger(AxoObjectEditor.class.getName()).log(Level.SEVERE, null, ex);
        }
        rSyntaxTextAreaXML.setText(os.toString());
        rSyntaxTextAreaXML.setSyntaxEditingStyle(SyntaxConstants.SYNTAX_STYLE_XML);
        rSyntaxTextAreaXML.setCodeFoldingEnabled(true);

        AxoObjectInstance obji = editObj.CreateInstance(null, "test", new Point(0, 0));
        jPanelKRateCode1.setText(obji.GenerateDoFunctionPlusPlus("", "", false));
        jPanelKRateCode1.setFont(jTextAreaKRateCode.getFont());
    }

    public void Close() {
        // warn if changes, and its not an embedded object

        if (modified && editObj.sPath != null) {
            if(jMenuItemSave.isEnabled()) {
                int result = JOptionPane.showConfirmDialog(this, "Unsaved changes, do you want to save?",
                    "Close", JOptionPane.YES_NO_CANCEL_OPTION);
                switch(result) {
                    case JOptionPane.CANCEL_OPTION:
                        return;
                    case JOptionPane.YES_OPTION:
                        jMenuItemSaveActionPerformed(null);
                        // fall through to close
                    case JOptionPane.NO_OPTION:
                    default:
                        ;
                }
            } else {
                int result = JOptionPane.showConfirmDialog(this, "Unsaved changes, do you want to add to a library?",
                    "Close", JOptionPane.YES_NO_CANCEL_OPTION);
                switch(result) {
                    case JOptionPane.CANCEL_OPTION:
                        return;
                    case JOptionPane.YES_OPTION:
                        jMenuItemAddToLibraryActionPerformed(null);
                        // this will currently call close(), but mod = false
                        return;
                    case JOptionPane.NO_OPTION:
                    default:
                        ;
                }
            }
        }
        DocumentWindowList.UnregisterWindow(this);
        editObj.removeObjectModifiedListener(this);
        dispose();
        origObj.CloseEditor();
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        jInternalFrame1 = new javax.swing.JInternalFrame();
        jLabel4 = new javax.swing.JLabel();
        jPanel1 = new javax.swing.JPanel();
        jTabbedPane1 = new javax.swing.JTabbedPane();
        jPanelOverview = new javax.swing.JPanel();
        jPanel2 = new javax.swing.JPanel();
        jLabel7 = new javax.swing.JLabel();
        jLabelName = new javax.swing.JLabel();
        jLabel8 = new javax.swing.JLabel();
        jTextFieldAuthor = new javax.swing.JTextField();
        jLabel9 = new javax.swing.JLabel();
        jTextFieldLicense = new javax.swing.JTextField();
        jPanel3 = new javax.swing.JPanel();
        jLabel10 = new javax.swing.JLabel();
        jScrollPane13 = new javax.swing.JScrollPane();
        jTextDesc = new javax.swing.JTextArea();
        jLabel5 = new javax.swing.JLabel();
        jScrollPane4 = new javax.swing.JScrollPane();
        jListIncludes = new javax.swing.JList();
        jLabel6 = new javax.swing.JLabel();
        jScrollPane12 = new javax.swing.JScrollPane();
        jListDepends = new javax.swing.JList();
        inletDefinitionsEditor1 = new axoloti.objecteditor.InletDefinitionsEditorPanel();
        outletDefinitionsEditorPanel1 = new axoloti.objecteditor.OutletDefinitionsEditorPanel();
        attributeDefinitionsEditorPanel1 = new axoloti.objecteditor.AttributeDefinitionsEditorPanel();
        paramDefinitionsEditorPanel1 = new axoloti.objecteditor.ParamDefinitionsEditorPanel();
        displayDefinitionsEditorPanel1 = new axoloti.objecteditor.DisplayDefinitionsEditorPanel();
        jPanelLocalData = new javax.swing.JPanel();
        jPanelInitCode = new javax.swing.JPanel();
        jPanelKRateCode = new javax.swing.JPanel();
        jPanelKRateCode1 = new javax.swing.JLabel();
        jPanelKRateCode2 = new javax.swing.JPanel();
        jPanelSRateCode = new javax.swing.JPanel();
        jPanelDisposeCode = new javax.swing.JPanel();
        jPanelMidiCode = new javax.swing.JPanel();
        jLabelMidiPrototype = new javax.swing.JLabel();
        jPanelMidiCode2 = new javax.swing.JPanel();
        jPanelXML = new javax.swing.JPanel();
        jScrollPane6 = new javax.swing.JScrollPane();
        rSyntaxTextAreaXML = new org.fife.ui.rsyntaxtextarea.RSyntaxTextArea();
        jLabel2 = new javax.swing.JLabel();
        jMenuBar1 = new javax.swing.JMenuBar();
        fileMenu1 = new axoloti.menus.FileMenu();
        jSeparator1 = new javax.swing.JPopupMenu.Separator();
        jMenuItemSave = new javax.swing.JMenuItem();
        jMenuItemAddToLibrary = new javax.swing.JMenuItem();
        jMenuItemApply = new javax.swing.JMenuItem();
        windowMenu1 = new axoloti.menus.WindowMenu();
        helpMenu1 = new axoloti.menus.HelpMenu();

        jInternalFrame1.setVisible(true);

        javax.swing.GroupLayout jInternalFrame1Layout = new javax.swing.GroupLayout(jInternalFrame1.getContentPane());
        jInternalFrame1.getContentPane().setLayout(jInternalFrame1Layout);
        jInternalFrame1Layout.setHorizontalGroup(
            jInternalFrame1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 0, Short.MAX_VALUE)
        );
        jInternalFrame1Layout.setVerticalGroup(
            jInternalFrame1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 0, Short.MAX_VALUE)
        );

        setDefaultCloseOperation(javax.swing.WindowConstants.DO_NOTHING_ON_CLOSE);
        setPreferredSize(new java.awt.Dimension(540, 400));
        addWindowListener(new java.awt.event.WindowAdapter() {
            public void windowClosing(java.awt.event.WindowEvent evt) {
                formWindowClosing(evt);
            }
        });
        getContentPane().setLayout(new javax.swing.BoxLayout(getContentPane(), javax.swing.BoxLayout.PAGE_AXIS));
        getContentPane().add(jLabel4);

        jPanel1.setPreferredSize(new java.awt.Dimension(640, 100));
        jPanel1.setLayout(new javax.swing.BoxLayout(jPanel1, javax.swing.BoxLayout.PAGE_AXIS));

        jTabbedPane1.setTabPlacement(javax.swing.JTabbedPane.LEFT);
        jTabbedPane1.setPreferredSize(new java.awt.Dimension(640, 100));

        jPanelOverview.setLayout(new javax.swing.BoxLayout(jPanelOverview, javax.swing.BoxLayout.Y_AXIS));

        jPanel2.setLayout(new java.awt.GridLayout(3, 2));

        jLabel7.setText("Name:");
        jPanel2.add(jLabel7);

        jLabelName.setText("object name");
        jPanel2.add(jLabelName);

        jLabel8.setText("Author:");
        jPanel2.add(jLabel8);

        jTextFieldAuthor.setText("jTextField1");
        jTextFieldAuthor.addFocusListener(new java.awt.event.FocusAdapter() {
            public void focusLost(java.awt.event.FocusEvent evt) {
                jTextFieldAuthorFocusLost(evt);
            }
        });
        jPanel2.add(jTextFieldAuthor);

        jLabel9.setText("License:");
        jPanel2.add(jLabel9);

        jTextFieldLicense.setText("jTextField2");
        jPanel2.add(jTextFieldLicense);

        jPanelOverview.add(jPanel2);

        jPanel3.setLayout(new javax.swing.BoxLayout(jPanel3, javax.swing.BoxLayout.Y_AXIS));

        jLabel10.setText("Description:");
        jPanel3.add(jLabel10);

        jTextDesc.setColumns(20);
        jTextDesc.setLineWrap(true);
        jTextDesc.setRows(5);
        jTextDesc.setWrapStyleWord(true);
        jScrollPane13.setViewportView(jTextDesc);

        jPanel3.add(jScrollPane13);

        jLabel5.setText("Includes");
        jPanel3.add(jLabel5);

        jListIncludes.setModel(new DefaultListModel());
        jScrollPane4.setViewportView(jListIncludes);

        jPanel3.add(jScrollPane4);

        jLabel6.setText("Dependencies");
        jPanel3.add(jLabel6);

        jListDepends.setModel(new DefaultListModel());
        jScrollPane12.setViewportView(jListDepends);

        jPanel3.add(jScrollPane12);

        jPanelOverview.add(jPanel3);

        jTabbedPane1.addTab("Overview", jPanelOverview);

        javax.swing.GroupLayout inletDefinitionsEditor1Layout = new javax.swing.GroupLayout(inletDefinitionsEditor1);
        inletDefinitionsEditor1.setLayout(inletDefinitionsEditor1Layout);
        inletDefinitionsEditor1Layout.setHorizontalGroup(
            inletDefinitionsEditor1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        inletDefinitionsEditor1Layout.setVerticalGroup(
            inletDefinitionsEditor1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("Inlets", inletDefinitionsEditor1);

        javax.swing.GroupLayout outletDefinitionsEditorPanel1Layout = new javax.swing.GroupLayout(outletDefinitionsEditorPanel1);
        outletDefinitionsEditorPanel1.setLayout(outletDefinitionsEditorPanel1Layout);
        outletDefinitionsEditorPanel1Layout.setHorizontalGroup(
            outletDefinitionsEditorPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        outletDefinitionsEditorPanel1Layout.setVerticalGroup(
            outletDefinitionsEditorPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("Outlets", outletDefinitionsEditorPanel1);

        javax.swing.GroupLayout attributeDefinitionsEditorPanel1Layout = new javax.swing.GroupLayout(attributeDefinitionsEditorPanel1);
        attributeDefinitionsEditorPanel1.setLayout(attributeDefinitionsEditorPanel1Layout);
        attributeDefinitionsEditorPanel1Layout.setHorizontalGroup(
            attributeDefinitionsEditorPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        attributeDefinitionsEditorPanel1Layout.setVerticalGroup(
            attributeDefinitionsEditorPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("Attributes", attributeDefinitionsEditorPanel1);

        javax.swing.GroupLayout paramDefinitionsEditorPanel1Layout = new javax.swing.GroupLayout(paramDefinitionsEditorPanel1);
        paramDefinitionsEditorPanel1.setLayout(paramDefinitionsEditorPanel1Layout);
        paramDefinitionsEditorPanel1Layout.setHorizontalGroup(
            paramDefinitionsEditorPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        paramDefinitionsEditorPanel1Layout.setVerticalGroup(
            paramDefinitionsEditorPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("Parameters", paramDefinitionsEditorPanel1);

        javax.swing.GroupLayout displayDefinitionsEditorPanel1Layout = new javax.swing.GroupLayout(displayDefinitionsEditorPanel1);
        displayDefinitionsEditorPanel1.setLayout(displayDefinitionsEditorPanel1Layout);
        displayDefinitionsEditorPanel1Layout.setHorizontalGroup(
            displayDefinitionsEditorPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        displayDefinitionsEditorPanel1Layout.setVerticalGroup(
            displayDefinitionsEditorPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("Displays", displayDefinitionsEditorPanel1);

        javax.swing.GroupLayout jPanelLocalDataLayout = new javax.swing.GroupLayout(jPanelLocalData);
        jPanelLocalData.setLayout(jPanelLocalDataLayout);
        jPanelLocalDataLayout.setHorizontalGroup(
            jPanelLocalDataLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        jPanelLocalDataLayout.setVerticalGroup(
            jPanelLocalDataLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("Local Data", jPanelLocalData);

        javax.swing.GroupLayout jPanelInitCodeLayout = new javax.swing.GroupLayout(jPanelInitCode);
        jPanelInitCode.setLayout(jPanelInitCodeLayout);
        jPanelInitCodeLayout.setHorizontalGroup(
            jPanelInitCodeLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        jPanelInitCodeLayout.setVerticalGroup(
            jPanelInitCodeLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("Init Code", jPanelInitCode);

        jPanelKRateCode.setLayout(new javax.swing.BoxLayout(jPanelKRateCode, javax.swing.BoxLayout.Y_AXIS));

        jPanelKRateCode1.setText("jLabel11");
        jPanelKRateCode.add(jPanelKRateCode1);

        javax.swing.GroupLayout jPanelKRateCode2Layout = new javax.swing.GroupLayout(jPanelKRateCode2);
        jPanelKRateCode2.setLayout(jPanelKRateCode2Layout);
        jPanelKRateCode2Layout.setHorizontalGroup(
            jPanelKRateCode2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        jPanelKRateCode2Layout.setVerticalGroup(
            jPanelKRateCode2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 288, Short.MAX_VALUE)
        );

        jPanelKRateCode.add(jPanelKRateCode2);

        jTabbedPane1.addTab("K-rate Code", jPanelKRateCode);

        javax.swing.GroupLayout jPanelSRateCodeLayout = new javax.swing.GroupLayout(jPanelSRateCode);
        jPanelSRateCode.setLayout(jPanelSRateCodeLayout);
        jPanelSRateCodeLayout.setHorizontalGroup(
            jPanelSRateCodeLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        jPanelSRateCodeLayout.setVerticalGroup(
            jPanelSRateCodeLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("S-rate Code", jPanelSRateCode);

        javax.swing.GroupLayout jPanelDisposeCodeLayout = new javax.swing.GroupLayout(jPanelDisposeCode);
        jPanelDisposeCode.setLayout(jPanelDisposeCodeLayout);
        jPanelDisposeCodeLayout.setHorizontalGroup(
            jPanelDisposeCodeLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        jPanelDisposeCodeLayout.setVerticalGroup(
            jPanelDisposeCodeLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("Dispose Code", jPanelDisposeCode);

        jPanelMidiCode.setLayout(new javax.swing.BoxLayout(jPanelMidiCode, javax.swing.BoxLayout.Y_AXIS));

        jLabelMidiPrototype.setText("jLabel11");
        jPanelMidiCode.add(jLabelMidiPrototype);

        javax.swing.GroupLayout jPanelMidiCode2Layout = new javax.swing.GroupLayout(jPanelMidiCode2);
        jPanelMidiCode2.setLayout(jPanelMidiCode2Layout);
        jPanelMidiCode2Layout.setHorizontalGroup(
            jPanelMidiCode2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 466, Short.MAX_VALUE)
        );
        jPanelMidiCode2Layout.setVerticalGroup(
            jPanelMidiCode2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 288, Short.MAX_VALUE)
        );

        jPanelMidiCode.add(jPanelMidiCode2);

        jTabbedPane1.addTab("MIDI Code", jPanelMidiCode);

        jPanelXML.addFocusListener(new java.awt.event.FocusAdapter() {
            public void focusGained(java.awt.event.FocusEvent evt) {
                jPanelXMLFocusGained(evt);
            }
        });

        rSyntaxTextAreaXML.setColumns(20);
        rSyntaxTextAreaXML.setRows(5);
        jScrollPane6.setViewportView(rSyntaxTextAreaXML);

        javax.swing.GroupLayout jPanelXMLLayout = new javax.swing.GroupLayout(jPanelXML);
        jPanelXML.setLayout(jPanelXMLLayout);
        jPanelXMLLayout.setHorizontalGroup(
            jPanelXMLLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(jScrollPane6, javax.swing.GroupLayout.DEFAULT_SIZE, 466, Short.MAX_VALUE)
        );
        jPanelXMLLayout.setVerticalGroup(
            jPanelXMLLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(jScrollPane6, javax.swing.GroupLayout.DEFAULT_SIZE, 302, Short.MAX_VALUE)
        );

        jTabbedPane1.addTab("XML", jPanelXML);

        jPanel1.add(jTabbedPane1);

        jLabel2.setHorizontalTextPosition(javax.swing.SwingConstants.LEADING);
        jPanel1.add(jLabel2);

        getContentPane().add(jPanel1);

        fileMenu1.setText("File");
        fileMenu1.add(jSeparator1);

        jMenuItemSave.setText("Save");
        jMenuItemSave.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jMenuItemSaveActionPerformed(evt);
            }
        });
        fileMenu1.add(jMenuItemSave);

        jMenuItemAddToLibrary.setText("Add to Library...");
        jMenuItemAddToLibrary.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jMenuItemAddToLibraryActionPerformed(evt);
            }
        });
        fileMenu1.add(jMenuItemAddToLibrary);

        jMenuItemApply.setText("Apply");
        jMenuItemApply.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                jMenuItemApplyActionPerformed(evt);
            }
        });
        fileMenu1.add(jMenuItemApply);

        jMenuBar1.add(fileMenu1);
        jMenuBar1.add(windowMenu1);

        helpMenu1.setText("Help");
        jMenuBar1.add(helpMenu1);

        setJMenuBar(jMenuBar1);

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void formWindowClosing(java.awt.event.WindowEvent evt) {//GEN-FIRST:event_formWindowClosing
        Close();
    }//GEN-LAST:event_formWindowClosing

    private void jPanelXMLFocusGained(java.awt.event.FocusEvent evt) {//GEN-FIRST:event_jPanelXMLFocusGained

    }//GEN-LAST:event_jPanelXMLFocusGained

    private void jTextFieldAuthorFocusLost(java.awt.event.FocusEvent evt) {//GEN-FIRST:event_jTextFieldAuthorFocusLost

    }//GEN-LAST:event_jTextFieldAuthorFocusLost

    private void jMenuItemSaveActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jMenuItemSaveActionPerformed
        applyChangesToOriginal();
        origObj.FireObjectModified(this);
        MainFrame.axoObjects.WriteAxoObject(editObj.sPath, editObj);
        modified = false;
    }//GEN-LAST:event_jMenuItemSaveActionPerformed

    private void jMenuItemAddToLibraryActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jMenuItemAddToLibraryActionPerformed
        applyChangesToEdit();
        AddToLibraryDlg dlg = new AddToLibraryDlg(this, true, editObj);
        dlg.setVisible(true);
        modified = false; 
        Close();
    }//GEN-LAST:event_jMenuItemAddToLibraryActionPerformed

    private void jMenuItemApplyActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_jMenuItemApplyActionPerformed
        applyChangesToOriginal();
        origObj.FireObjectModified(this);
        // dont clear modified, so they are still prompted to save.
    }//GEN-LAST:event_jMenuItemApplyActionPerformed

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private axoloti.objecteditor.AttributeDefinitionsEditorPanel attributeDefinitionsEditorPanel1;
    private axoloti.objecteditor.DisplayDefinitionsEditorPanel displayDefinitionsEditorPanel1;
    private axoloti.menus.FileMenu fileMenu1;
    private axoloti.menus.HelpMenu helpMenu1;
    private axoloti.objecteditor.InletDefinitionsEditorPanel inletDefinitionsEditor1;
    private javax.swing.JInternalFrame jInternalFrame1;
    private javax.swing.JLabel jLabel10;
    private javax.swing.JLabel jLabel2;
    private javax.swing.JLabel jLabel4;
    private javax.swing.JLabel jLabel5;
    private javax.swing.JLabel jLabel6;
    private javax.swing.JLabel jLabel7;
    private javax.swing.JLabel jLabel8;
    private javax.swing.JLabel jLabel9;
    private javax.swing.JLabel jLabelMidiPrototype;
    private javax.swing.JLabel jLabelName;
    private javax.swing.JList jListDepends;
    private javax.swing.JList jListIncludes;
    private javax.swing.JMenuBar jMenuBar1;
    private javax.swing.JMenuItem jMenuItemAddToLibrary;
    private javax.swing.JMenuItem jMenuItemApply;
    private javax.swing.JMenuItem jMenuItemSave;
    private javax.swing.JPanel jPanel1;
    private javax.swing.JPanel jPanel2;
    private javax.swing.JPanel jPanel3;
    private javax.swing.JPanel jPanelDisposeCode;
    private javax.swing.JPanel jPanelInitCode;
    private javax.swing.JPanel jPanelKRateCode;
    private javax.swing.JLabel jPanelKRateCode1;
    private javax.swing.JPanel jPanelKRateCode2;
    private javax.swing.JPanel jPanelLocalData;
    private javax.swing.JPanel jPanelMidiCode;
    private javax.swing.JPanel jPanelMidiCode2;
    private javax.swing.JPanel jPanelOverview;
    private javax.swing.JPanel jPanelSRateCode;
    private javax.swing.JPanel jPanelXML;
    private javax.swing.JScrollPane jScrollPane12;
    private javax.swing.JScrollPane jScrollPane13;
    private javax.swing.JScrollPane jScrollPane4;
    private javax.swing.JScrollPane jScrollPane6;
    private javax.swing.JPopupMenu.Separator jSeparator1;
    private javax.swing.JTabbedPane jTabbedPane1;
    private javax.swing.JTextArea jTextDesc;
    private javax.swing.JTextField jTextFieldAuthor;
    private javax.swing.JTextField jTextFieldLicense;
    private axoloti.objecteditor.OutletDefinitionsEditorPanel outletDefinitionsEditorPanel1;
    private axoloti.objecteditor.ParamDefinitionsEditorPanel paramDefinitionsEditorPanel1;
    private org.fife.ui.rsyntaxtextarea.RSyntaxTextArea rSyntaxTextAreaXML;
    private axoloti.menus.WindowMenu windowMenu1;
    // End of variables declaration//GEN-END:variables

    @Override
    public JFrame GetFrame() {
        return this;
    }

    @Override
    public boolean AskClose() {
        Close();
        return false; //TBC
    }

    @Override
    public void ObjectModified(Object src) {
        FireObjectModified();
    }

    @Override
    public File getFile() {
        return null;
    }

    @Override
    public ArrayList<DocumentWindow> GetChildDocuments() {
        return null;
    }
}
