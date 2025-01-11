package dev.nullrobotics.choreolib.dairy.commands;

import org.jetbrains.annotations.NotNull;

import java.util.Collections;
import java.util.Set;

import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Command;

public class ChoreoCommand implements Command {
    @Override
    public void initialise() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean b) {

    }

    @Override
    public boolean finished() {
        return false;
    }

    @NotNull
    @Override
    public Set<Object> getRequirements() {
        return Collections.emptySet();
    }

    @NotNull
    @Override
    public Set<Wrapper.OpModeState> getRunStates() {
        return Collections.emptySet();
    }
}
