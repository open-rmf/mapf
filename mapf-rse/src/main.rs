use bevy::prelude::{App, Entity, Query, Res};
use librmf_site_editor::{
    site::NameOfSite, widgets::prelude::*, workspace::CurrentWorkspace, SiteEditor,
};

#[derive(SystemParam)]
pub struct HelloSiteWidget<'w, 's> {
    sites: Query<'w, 's, &'static NameOfSite>,
    current: Res<'w, CurrentWorkspace>,
}

impl<'w, 's> WidgetSystem<Tile> for HelloSiteWidget<'w, 's> {
    fn show(_: Tile, ui: &mut Ui, state: &mut SystemState<Self>, world: &mut World) {
        let mut params = state.get_mut(world);
        if let Some(name) = params
            .current
            .root
            .map(|e| params.sites.get(e).ok())
            .flatten()
        {
            ui.separator();
            ui.add_space(50.0);
            ui.heading(format!("MAPF Settings"));
        }
    }
}

fn main() {
    let mut app = App::new();
    app.add_plugins((
        SiteEditor::default(),
        PropertiesTilePlugin::<HelloSiteWidget>::new(),
    ));

    app.run();
}
