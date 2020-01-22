import * as core from '@actions/core'
import {execBashCommand} from '../src/action-ros-ci'

describe('execBashCommand test suite', () => {
  it('calls coreGroup', async () => {
      const mockGroup = jest.spyOn(core, 'group');
      const result = await execBashCommand('echo "Hello World"');
      expect(mockGroup).toBeCalled();
      expect(result).toEqual(0);
  });
  it('uses a prefix', async () => {
    const mockGroup = jest.spyOn(core, 'group');
    const result = await execBashCommand('Hello World', 'echo ');
    expect(mockGroup).toBeCalled();
    expect(result).toEqual(0);
  });
  it('ignores return code', async () => {
    const mockGroup = jest.spyOn(core, 'group');
    const options = {
      ignoreReturnCode: true
    }
    const result = execBashCommand('somebadcommand', '', options);
    expect(mockGroup).toBeCalled();
    expect(result).not.toEqual(0);
  })
})
