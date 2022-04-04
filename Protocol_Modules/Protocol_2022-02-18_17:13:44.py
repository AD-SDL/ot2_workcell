import error_handling
import opentrons.simulate
protocol = opentrons.simulate.get_protocol_api('2.8')

if __name__ == "__main__":

    tiprack_type = "opentrons_96_tiprack_20ul"
    PIPETTE_TYPE_single = 'p20_single_gen2'
    PIPETTE_MOUNT_single = 'right'
    DESTINATION_PLATE_TYPE = 'nest_96_wellplate_100ul_pcr_full_skirt'
    plate = protocol.load_labware('nest_96_wellplate_100ul_pcr_full_skirt', 4)
    tiprack_1 = protocol.load_labware('opentrons_96_tiprack_20ul', 3)
    p20 = protocol.load_instrument(PIPETTE_TYPE_single, PIPETTE_MOUNT_single, tip_racks=[tiprack_1])
    p20.pick_up_tip()
    error_handling.is_tip_attached(p20, protocol)


    p20.aspirate(20, plate['A1'])
    p20.dispense(20, plate['B1'])
    p20.drop_tip()
    error_handling.is_tip_dropped(p20, protocol)
    # #Fix the problem
    # protocol.resume()
    for commands in protocol.commands():
        print(commands)